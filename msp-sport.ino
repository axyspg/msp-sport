/********************************************************************************************************
 * MSP to SmartPort Telemetry
 * 
 * This program is to send incoming MSP telemetry data via SmartPort to a compatible receiver.
 * Also can be used as an ISP programmer for the MinimOSD board.
 * 
 * Telemetry has been tested on FrSky X8R receiver and Taranis X9D+ transmitter.
 * 
 * More info at http://someonedidthatalready.blogspot.com
 * 
 *  
 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or any later version. see http://www.gnu.org/licenses/
 * 
 * This program has lots code from MWOSD (Scarab NG OSD). Thanks a lot for your efforts!
 *  
 *  
 * MWOSD      Copyright (c) ShikOfTheRa (http://www.mwosd.com/)
 *                      GNU GPL 3 License http://www.gnu.org/licenses/
 *                      
 * ArduinoISP Copyright (c) 2008-2011 Randall Bohn (https://github.com/rsbohn/ArduinoISP)
 *                      BSD License http://www.opensource.org/licenses/bsd-license.php
 *            
 * FrSkySport Library   (c) Pawelsky 20150725 (http://www.rcgroups.com/forums/showthread.php?t=2245978)
 *                      Not for commercial use
 *                      
 * SoftSerial Library   (c) Mikal Hart (http://arduiniana.org/libraries/newsoftserial/)
 * 
 *
 * ver. 0.16  First release
 *
 *                      
 * MinimOSD Rx ---> ProMini Rx                     
 * ProMini D2 --> 4k7 --> Receiver S.Port  
 * GND -- GND
 * 5V -- 5V
 * No other.                     
 *                      
 */


 /********************       WARNING/STATUS settings      *********************/
//#define GPSACTIVECHECK 5            // Alerts if no GPS data for more than x secs. Sets GPS sats to zero
//#define MSPACTIVECHECK 3            // Alerts if no Flight controller data for more than x secs. 
//#define INCANGLE       25           // Alerts if craft's angle is greater or equal to this value

/******************** Serial speed settings *********************/
// Choose ONLY ONE option: (match with CleanFlight and MWOSD baudrate)
#define BAUDRATE 115200
//#define BAUDRATE 57600
//#define BAUDRATE 38400
//#define BAUDRATE 19200
//#define BAUDRATE 9600

 
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include "FrSkySportSensor.h"
//#include "FrSkySportSensorAss.h"
#include "FrSkySportSensorFcs.h"
//#include "FrSkySportSensorFlvss.h"
#include "FrSkySportSensorGps.h"
//#include "FrSkySportSensorSp2uart.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#include "Config.h"
#include "GlobalVariables.h"

#ifdef CFTS_ENABLED
#include "FrSkySportSensorRpmMod.h"
  class FrSkySportStatus: public FrSkySportSensorRpmMod
  {
    public:
      void send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
      
    private:
  };
  FrSkySportStatus sp_status;
#else
#include "FrSkySportSensorRpm.h"
  FrskySportSensorRpm sp_status;
#endif

FrSkySportTelemetry telemetry;
FrSkySportSensorFcs vfas;
FrSkySportSensorGps gps;
FrSkySportSensorVario vario;

//------------------------------------------------------------------------
void setup()
{
  unsigned long elapsed = millis();

  // keep MinimOSD reset
  pinMode(MINIMOSD_RST, OUTPUT);
  digitalWrite(MINIMOSD_RST, LOW);  

#ifdef ISP_MODE
  // Enter in ISP mode for 5 seconds

  // save registers
  uint8_t a, b ,c;  
  a = SPCR;
  b = SPSR;
  c = SPDR;
  setup_isp();

  // if no data received for X seconds, exit ISP mode
  while (millis() - elapsed < ISP_TIMEOUT) if (loop_isp()) elapsed = millis(); // if data is received, extend time ISP mode time
  
  // restore registers
  SPCR = a;
  SPSR = b;
  SPDR = c;

  // reconfigure pins used by isp mode
  pinMode(MINIMOSD_RST, OUTPUT);
  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
#endif

  // Initialize MSP to SmartPort port
  Serial.begin(BAUDRATE);
//---- override UBRR with MWC settings
/*  uint8_t h = ((F_CPU  / 4 / (BAUDRATE) -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / (BAUDRATE) -1) / 2);
  UCSR0A  |= (1<<U2X0); UBRR0H = h; UBRR0L = l; */
//---
  Serial.flush();

  // Initialize SmartPort
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_2, &vfas, &gps, &vario, &sp_status);

  // we are ready, so let MinimOSD start
  digitalWrite(MINIMOSD_RST, HIGH);  
}

//------------------------------------------------------------------------
void loop()
{
  if(millis() > timer.seconds+1000)     // this execute 1 time a second
  {
    timer.seconds+=1000;
//    timer.tenthSec=0;
//    onTime++;
    #ifdef GPSACTIVECHECK
      if (timer.GPS_active==0){
        GPS_numSat=0;
      }
      else {
        timer.GPS_active--;
      }      
    #endif // GPSACTIVECHECK 
    if (timer.MSP_active>0){
      timer.MSP_active--;
    }  
  }
  
  // Receive and process MSP protocol
  serialMSPreceive(1);
  
  // Send Smartport data telemetry to receiver if MSP did NOT timeout
  if(timer.MSP_active==0) telemetry.reset();
    else telemetry.send();  
}

//------------------------------------------------------------------------
void FrSkySportStatus::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  if(sensorId != id) return;

#ifdef CFTS_ENABLED
  static uint32_t sid = 0;
  
  static int incr = 20;
  static int val = HIGH;
  
  incr++;
  if (incr >= 20)
  {
    incr = 0;
//    digitalWrite(LEDPIN, val);
    val = (val ? LOW : HIGH);
  }

  sid++;
  if (sid > 2) sid = 0;

  uint32_t packet = 0;
  
  switch(sid)
  {
      case 0: // check point, protocol and sensors info
          packet = (MwProfile & 0x0F);
          packet |= 0x10; // Most significant word of last byte is protocol version and least significant is profile number 
          packet |= (MwSensorPresent << 8); // Sensors
          break;

      case 1: // mode flags (boxes)
          packet = MwSensorActive;
          break;

      case 2: // num gps and status flags
          MwStateFlags = 0x00;
          if (GPS_fix) MwStateFlags |= 0x02;
          // too much inclination triggers INC alert
          if (abs(MwAngle[0] / 10) >= INCANGLE || abs(MwAngle[1] / 10) >= INCANGLE) MwStateFlags |= 0x08;
          
          packet = GPS_numSat | (MwStateFlags << 8);// | (MwArmedFlags << 16);
          break;
  }

  packet = packet & 0x0FFFFFFF;
  packet = packet | (sid << 28);

  t2 = packet;
  // Force sending of T2 value (this skips t1 and rpm values!)
  t2Time = 0;
  sensorDataIdx = 1;

  FrSkySportSensorRpmMod::send(serial, id, now);
#endif
}


