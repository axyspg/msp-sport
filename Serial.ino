
#define SERIALBUFFERSIZE 150

static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;
uint8_t txChecksum;

uint32_t read32() {
  uint32_t t = read16();
  t |= (uint32_t)read16()<<16;
  return t;
}

uint16_t read16() {
  uint16_t t = read8();
  t |= (uint16_t)read8()<<8;
  return t;
}

uint8_t read8()  {
  return serialBuffer[readIndex++];
}

void serialMSPreceive(uint8_t loops)
{
  uint8_t c;
  uint8_t loopserial=0;

  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  }
  c_state = IDLE;

  if (Serial.available()) loopserial=1;
  while(loopserial==1)
  {
    c = Serial.read();

    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if (c_state == HEADER_ARROW)
    {
      if (c > SERIALBUFFERSIZE)
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        dataSize = c;
        c_state = HEADER_SIZE;
        rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      rcvChecksum ^= c;
      receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(rcvChecksum == 0) {
            serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
    if (loops==0) loopserial=0;
    if (!Serial.available()) loopserial=0;
  }
}


// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;
  #ifdef MSPACTIVECHECK
    timer.MSP_active=MSPACTIVECHECK; // getting something on serial port
  #endif

  if (cmdMSP==MSP_STATUS)
  {
    cycleTime=read16();
    I2CError=read16();
    MwSensorPresent = read16();
    MwSensorActive = read32();
/*    #if defined FORCESENSORS
      MwSensorPresent=GPSSENSOR|BAROMETER|MAGNETOMETER|ACCELEROMETER;
    #endif  */
    MwProfile = read8();
/*    armed = (MwSensorActive & mode.armed) != 0;
    MwStateFlags = read8();
    MwArmedFlags = read8();*/
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    #ifdef GPSACTIVECHECK
     timer.GPS_active=GPSACTIVECHECK;
    #endif //GPSACTIVECHECK
    uint8_t GPS_fix_temp=read8();
    if (GPS_fix_temp){
      GPS_fix=1;
    }
    GPS_numSat=read8();
    GPS_latitude = read32();
    GPS_longitude = read32();
    GPS_altitude = read16();
    #if defined RESETGPSALTITUDEATARM
      if (!armed){
        GPS_home_altitude=GPS_altitude;
      } 
      GPS_altitude=GPS_altitude-GPS_home_altitude;
    #endif // RESETGPSALTITUDEATARM  
    #if defined I2CGPS_SPEED
      GPS_speed = read16()*10;
      //gpsfix(); untested
    #else
      GPS_speed = read16();
    #endif // I2CGPS_SPEED
    GPS_ground_course = read16();

//    gps.setData(GPS_latitude, GPS_longitude, GPS_altitude, GPS_speed, GPS_ground_course, 0, 0, 0, 0, 0, 0); // use GPS heading
    gps.setData(GPS_latitude, GPS_longitude, GPS_altitude, GPS_speed, MwHeading, 0, 0, 0, 0, 0, 0);  // use Magnetometer heading
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    GPS_distanceToHome=read16();
    #ifdef I2CGPS_DISTANCE
      gpsdistancefix();
    #endif
    
    GPS_directionToHome=read16();
    read8(); //missing
    GPS_time = read32();        //local time of coord calc - haydent
    //Debug
    /*Serial.print(GPS_latitude);
    Serial.print("--");
    Serial.print(GPS_longitude);
    Serial.print("--");
    Serial.println(GPS_time);
    */
    
//    gps.setData((float)GPS_latitude/10000000.0f, (float)GPS_longitude/10000000.0f, GPS_altitude, (float)GPS_speed/100.0f, MwHeading, 0, 0, 0, 0, 0, 0);  // use Magnetometer heading
    if (GPS_fix && GPS_numSat > 5)
    {
      gps.setData(GPS_latitude/10000000.0, GPS_longitude/10000000.0, // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
                  GPS_altitude,                                      // Altitude in m (can be negative)
                  GPS_speed/100.0,                                   // Speed m/s
                  MwHeading,                                         // Course over ground in degrees (0-359, 0 = north), use Magnetometer heading
                  0,0,0,                                             // Date (year - 2000, month, day)
                  0,0,0);                                            // Time (hour, minute, second) - will be affected by timezone setings in your radio
    }
  }
  

  if (cmdMSP==MSP_ALTITUDE)
  {
    #if defined(USEGPSALTITUDE)
      MwAltitude = (int32_t)GPS_altitude*100;
      gpsvario();
    #else    
      MwAltitude = read32();
      MwVario = read16();
    #endif

    vario.setData(((float)MwAltitude / 100.0f), ((float)MwVario / 10.0f));
  }

  if (cmdMSP==MSP_ANALOG)
  {
    MwVBat=read8();
    pMeterSum=read16();
    MwRssi = read16();
    MWAmperage = read16();
    
    vfas.setData(((float)MWAmperage / 10.0f), ((float)MwVBat / 10.0f));
 }

  if (cmdMSP==MSP_ATTITUDE)
  {
    for(uint8_t i=0;i<2;i++){
      MwAngle[i] = read16();
    }
      MwHeading = read16();
    #if defined(USEGPSHEADING)
      MwHeading = GPS_ground_course/10;
    #endif
    #ifdef HEADINGCORRECT
      if (MwHeading >= 180) MwHeading -= 360;
    #endif
  }
}
// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------

