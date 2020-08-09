#define SERIALBUFFERSIZE 125

static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint16_t dataSize;
static uint16_t cmdMSP; // 8 for MSP or 16 for MSPV2
static uint8_t rcvChecksum;
static uint8_t readIndex;
static uint8_t txChecksum;

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

#define skip8() {readIndex++;}
#define skip16() {readIndex+=2;}
#define skip32() {readIndex+=4;}
#define skipn(n) {readIndex+=n;}


void serialMSPreceive(uint8_t loops)
{
  uint8_t c;
  static uint8_t flagMSP2 = 0;
  uint8_t loopserial = 0;
  static uint8_t MSPversion;
  static enum _serial_state {
      IDLE, 
      HEADER_START,
      HEADER_MX,
      HEADER_ARROW,
      HEADER_SIZE,
      PAYLOAD_READY,
#ifdef MSPV2
      HEADER_CMD1_MSPV2,
      HEADER_CMD2_MSPV2,
      HEADER_SIZE1_MSPV2,
#endif    
  } c_state = IDLE;

  if (Serial.available()) loopserial = 1;

  while (loopserial == 1)
  {
      c = Serial.read();

      if (c_state == IDLE)
      {
          c_state = (c == '$') ? HEADER_START : IDLE;
          MSPversion = 1;
      }
      else if (c_state == HEADER_START)
      {
          c_state = IDLE;
          if (c == 'M') {
              c_state = HEADER_MX;
          }
#ifdef MSPV2
          if (c == 'X') {
              c_state = HEADER_MX;
              MSPversion = 2;
              flagMSP2 = 0;
          }
#endif      
      }
      else if (c_state == HEADER_MX)
      {
          c_state = (c == '>') ? HEADER_ARROW : IDLE;
      }
      else if (c_state == HEADER_ARROW)
      {
          c_state = HEADER_SIZE;
#ifdef MSPV2
          if (MSPversion == 1) {
              dataSize = c;
              if (dataSize > SERIALBUFFERSIZE) {  // now we are expecting the payload size
                  c_state = IDLE;
              }
          }
          else flagMSP2 = c;
#else
          dataSize = c;
          if (c > SERIALBUFFERSIZE) {  // now we are expecting the payload size
              c_state = IDLE;
          }

#endif        
          rcvChecksum = 0;
          crc8_dvb_s2(rcvChecksum, c, MSPversion);
      }
      else if (c_state == HEADER_SIZE)
      {
#ifdef MSPV2
          cmdMSP = c;
          if (MSPversion == 2) {
              c_state = HEADER_CMD1_MSPV2;
          }
          else {
              receiverIndex = 0;
              c_state = PAYLOAD_READY;
          }
#else
          cmdMSP = c;
          c_state = PAYLOAD_READY;
#endif        
          crc8_dvb_s2(rcvChecksum, c, MSPversion);
          receiverIndex = 0;
      }

#ifdef MSPV2
      else if (c_state == HEADER_CMD1_MSPV2)
      {
          crc8_dvb_s2(rcvChecksum, c, MSPversion);
          cmdMSP += (uint16_t)(c << 8);
          c_state = HEADER_CMD2_MSPV2;
      }
      else if (c_state == HEADER_CMD2_MSPV2)
      {
          crc8_dvb_s2(rcvChecksum, c, MSPversion);
          dataSize = c;
          c_state = HEADER_SIZE1_MSPV2;
      }
      else if (c_state == HEADER_SIZE1_MSPV2)
      {
          crc8_dvb_s2(rcvChecksum, c, MSPversion);
          dataSize += (uint16_t)(c << 8);
          if (dataSize > SERIALBUFFERSIZE) {  // now we are expecting the payload size
              c_state = IDLE;
          }
          else {
              c_state = PAYLOAD_READY;
          }
          receiverIndex = 0;
      }
#endif      
      else if (c_state == PAYLOAD_READY) // ready for payload / cksum
      {
          if (receiverIndex == dataSize) // received checksum byte
          {            
              if (rcvChecksum == c) {
                  serialMSPCheck();
              }
              else {
                  gDebug1 = cmdMSP; // DEBUG
              }
              c_state = IDLE;
          }
          else {
              crc8_dvb_s2(rcvChecksum, c, MSPversion);
              serialBuffer[receiverIndex++] = c;
          }
      }
      if (loops == 0) loopserial = 0;

      if (!Serial.available()) loopserial = 0;
  }
}


void crc8_dvb_s2(uint8_t crc, unsigned char a, uint8_t crcversion)
{
  crc ^= a;
  if (crcversion == 2){   
    for (int ii = 0; ii < 8; ++ii){
      if (crc & 0x80){
        crc = (crc << 1) ^ 0xD5;
      }
      else{
        crc = crc << 1;
      }
    }
  }
  rcvChecksum = crc; //  return crc;
}


// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
void serialMSPCheck()
{
  readIndex = 0;
    
  if (cmdMSP == MSP_STATUS)
  {
    uint16_t cycleTime = read16();
    uint16_t I2CError  = read16();
    uint16_t MwSensorPresent = read16();
    gMwSensorActive  = read32();
/*    #if defined FORCESENSORS
      MwSensorPresent=GPSSENSOR|BAROMETER|MAGNETOMETER|ACCELEROMETER;
   // #endif  */
    uint8_t MwProfile = read8();
/*    armed = (MwSensorActive & mode.armed) != 0;
    MwStateFlags = read8();
    MwArmedFlags = read8();*/
  }

  // FSSP_DATAID_LATLONG    0x0800
  // FSSP_DATAID_GPS_ALT    0x0820
  // FSSP_DATAID_SPEED      0x0830
  if (cmdMSP == MSP_RAW_GPS)
  {
    gGPS_fix = read8();
    gGPS_sats = read8();

    gGPS_latitude = read32();
    gGPS_longitude = read32();
    gGPS_altitude = read16();
    gGPS_speed = read16();
    gGPS_groundCourse = read16();
    gGPS_hdop = read16();

//    gps.setData(GPS_latitude, GPS_longitude, GPS_altitude, GPS_speed, GPS_ground_course, 0, 0, 0, 0, 0, 0); // use GPS heading
//    gps.setData(gGPS_latitude, gGPS_longitude, gGPS_altitude, gGPS_speed, gHeading, 0, 0, 0, 0, 0, 0);  // use Magnetometer heading
    gps.setData(gGPS_latitude / 10000000.0, gGPS_longitude / 10000000.0, // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
                gGPS_altitude,                                           // Altitude in m (can be negative)
                gGPS_speed / 100.0,                                      // Speed m/s
                gHeading,                                                // Course over ground in degrees (0-359, 0 = north), use Magnetometer heading
                datetime.year, datetime.month, datetime.day,             // Date (year - 2000, month, day)
                datetime.hours, datetime.minutes, datetime.seconds);     // Time (hour, minute, second) - will be affected by timezone setings in your radio
  }


  // FSSP_DATAID_HOME_DIST  0x0420
  if (cmdMSP == MSP_COMP_GPS)
  {
    gGPS_distanceToHome = read16();
    gGPS_directionToHome = read16();
    read8();  //missing
    read32(); //local time of coord calc - haydent    
  }
  
/*  if (cmdMSP == MSP_GPSSTATISTICS)
  {
    read16(); // lastMessageDt
    read32(); // errors
    read32(); // timeouts
    read32(); // packetCount
    gGPS_hdop = read16(); // hdop
    read16(); // eph
    read16(); // epv
  }*/


  // VARIO - Vario Sensor
  // FSSP_DATAID_ALTITUDE 0x0100
  // FSSP_DATAID_VARIO    0x0110
  if (cmdMSP == MSP_ALTITUDE)
  {
    int32_t Altitude = read32();
    int16_t Vario    = read16();

    vario.setData(((float)Altitude / 100.0f), ((float)Vario / 10.0f));
  }


  // FSSP_DATAID_PITCH   0x0430
  // FSSP_DATAID_ROLL    0x0440
  // FSSP_DATAID_HEADING 0x0840
  // FSSP_DATAID_ACCX    0x0700
  // FSSP_DATAID_ACCY    0x0710
  // FSSP_DATAID_ACCZ    0x0720
  if (cmdMSP == MSP_ATTITUDE)
  {
    int16_t roll = read16();
    int16_t pitch = read16();
    gHeading = read16();
    
/*    #if defined(USEGPSHEADING)
      MwHeading = gGPS_groundCourse/10;
    #endif
    #ifdef HEADINGCORRECT
      if (gHeading >= 180) gHeading -= 360;
    #endif*/

    gyro.setData(-((float)pitch), ((float)roll), gDebug1, gDebug2, gDebug3);
  }

/*
  if (cmdMSP == MSP_CELLS)
  {
    gNumCells = 0;
    for(uint8_t i = 0; i < 6; i++) {
      gCell_data[i] = read16();
      if (gCell_data[i] > 0) gNumCells++;
    }

    static int incr = 0;

    float A4 = gCell_data[incr];
    incr++;
    if (incr == 6) A4 = gNumCells;
    if (incr > 6) incr = 0;

    if (gNumCells > 0) {
        A4 = ((float)gVBat / 10.0f) / (float)gNumCells;
    }

    sp2uart.setData(0, A4);    
  }*/


  // RTC from FC
  if (cmdMSP == MSP_RTC)
  {
    updateDateTime(read32());
  }


  if(cmdMSP == MSP_BOXIDS)
  {
    uint32_t bit = 1;
    uint8_t remaining = dataSize;
    memset(&mode, 0, sizeof(mode));

    while(remaining > 0) {
      uint8_t c = read8();
/*      #ifdef DEBUGDPOSMSPID    
        boxidarray[dataSize-remaining]=c;
      #endif  */
      switch(c) {
      case 0:
        mode.armed |= bit;
        break;
      case 1:
        mode.stable |= bit;
        break;
      case 2:
        mode.horizon |= bit;
        break;
      case 3:
        mode.baro |= bit;
        break;
      case 5:
        mode.mag |= bit;
        break;
      case 8:
        mode.camstab |= bit;
       break;
      case 10:
        mode.gpshome |= bit;
        break;
      case 11:
        mode.gpshold |= bit;
        break;
      case 12:
        mode.passthru  |= bit;
        break;
      case 16:
        mode.llights |= bit;
        break;
      case 19:
        mode.osd_switch |= bit;
        break;
      case IDBOXAIR:
        mode.air |= bit;
        break;
      case 27:
        mode.failsafe |= bit;
        break;
      case IDBOXWP:
        mode.gpsmission |= bit;
        break;
//#ifdef EXTENDEDMODESUPPORT
      case IDBOXGPSLAND:
        mode.gpsland |= bit;
        break;
      case 21:
        mode.autotune |= bit;
        break;
      case 37:
        mode.autotrim |= bit;
        break;
      case 36:
        mode.launch |= bit;
        break;
      case 45:
        mode.cruise |= bit;
        break;
      case 34:
        mode.flaperon |= bit;
        break;
//#endif // EXTENDEDMODESUPPORT
/*#if defined ACROPLUS
      case 29:
        mode.acroplus |= bit;
        break;
#endif //ACROPLUS        
*/
      }
      bit <<= 1;
      --remaining;
    }
  }


/*  if (cmdMSP == MSP_ANALOG)
  {
    gVBat = read8();
    uint16_t MeterSum = read16();
    uint16_t Rssi = read16();
    uint16_t Amperage = read16();
    
    vfcs.setData(((float)Amperage / 10.0f), ((float)gVBat / 10.0f));
  }*/
  
  // VFAS/FCS - Voltage / Amperage Sensor
  // A3 / A4 sensor
  // FSSP_DATAID_CURRENT 0x0200
  // FSSP_DATAID_VFAS    0x0210
  // FSSP_DATAID_FUEL    0x0600
  // FSSP_DATAID_A3      0x0900
  // FSSP_DATAID_A4      0x0910

  if (cmdMSP == MSP2_INAV_ANALOG)
  {
    uint8_t BatStatus = read8();  // battery status    
    gNumCells = (BatStatus >> 4);
    gVBat = read16(); // battery voltage
    uint16_t Amperage = read16(); // amperage
    gDebug2 = read32(); // power
    read32(); // MAh drawn
    read32(); // MWh drawn
    read32(); // battery remaining capacity
    uint8_t remain = read8();  // battery percentage
    gRssi = read16(); // RSSI

    vfcs.setData(((float)Amperage / 1000.0f), ((float)gVBat / 100.0f));
    //sp2uart.setData(gNumCells, ((float)gVBat / (float)gNumCells)); // DEBUG: send number of cells
    sp2uart.setData(0, ((float)gVBat / (float)gNumCells) / 100.0f);
    fuel.setData(remain);
  }

  // FSSP_DATAID_ASPD    0x0A00
  if (cmdMSP == MSP2_INAV_AIR_SPEED)
  {
    float t_AIR_speed = read32();
    gAIR_speed = t_AIR_speed;

    // TODO: send airspeed sensor
  }

  // TODO:  
  // FSSP_DATAID_FPV        0x0450

}
// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------


void updateDateTime(uint32_t t_time)
{
  //datetime.unixtime=1527712200; // 30/05/2018 @ 20:30 UTC for testing

  t_time -= 946684800;
  uint8_t  t_year=0;
  uint8_t  t_month=0;
  uint8_t  t_monthsize=0;
  uint32_t t_days=0;
  static const uint8_t daysinmonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

#define LEAP_YEAR(Y) !(((Y))%4) 
 // UTC time support only
/*#ifndef DATEFORMAT_UTC
  int32_t t_tzhours = 3600 * (128 - Settings[S_GPSTZ]);
  t_time = t_time - t_tzhours;
#endif // DATEFORMAT_UTC*/

  datetime.seconds = uint32_t (t_time % 60); t_time /= 60;
  datetime.minutes = uint32_t (t_time % 60); t_time /= 60;
  datetime.hours = uint32_t (t_time % 24);   t_time /= 24;

  while ((unsigned)(t_days += (LEAP_YEAR(t_year) ? 366 : 365)) <= t_time) {
    t_year++;
  }
  t_days -= LEAP_YEAR(t_year) ? 366 : 365;
  t_time  -= t_days;

  t_days = 0;
  t_month = 0;
  t_monthsize = 0;
  for (t_month = 0; t_month < 12; t_month++) {
    if (t_month == 1) { // february
      if (LEAP_YEAR(t_year)) {
        t_monthsize = 29;
      } else {
        t_monthsize = 28;
      }
    } else {
      t_monthsize = daysinmonth[t_month];
    }

    if (t_time >= t_monthsize) {
      t_time -= t_monthsize;
    } else {
      break;
    }
  }
#ifdef  DATEFORMAT_US
  datetime.day   = t_month + 1;
  datetime.month = t_time + 1;
#else
  datetime.day   = t_time + 1;
  datetime.month = t_month + 1;
#endif
  datetime.year  = t_year;
}
