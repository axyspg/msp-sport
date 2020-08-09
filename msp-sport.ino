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
 * ver. 0.20  Migrated to iNAV Lua Telemetry
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


#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include "FrSkySportSensor.h"
//#include "FrSkySportSensorAss.h"
#include "FrSkySportSensorFcs.h"
//#include "FrSkySportSensorFlvss.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSensorSp2uart.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#include "FrSkySportSensorGyro.h"
#include "FrSkySportSensorFuel.h"
#include "Config.h"
#include "GlobalVariables.h"

FrSkySportTelemetry telemetry;
FrSkySportSensorFcs vfcs;
FrSkySportSensorGps gps;
FrSkySportSensorRpm rpm;
FrSkySportSensorVario vario;
FrSkySportSensorSp2uart sp2uart;
FrSkySportSensorGyro gyro;
FrSkySportSensorFuel fuel;


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
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_2, &vfcs, &gps, &vario, &sp2uart, &rpm, &gyro, &fuel);

  // we are ready, so let MinimOSD start
  digitalWrite(MINIMOSD_RST, HIGH);  
}


//------------------------------------------------------------------------
void loop()
{
  uint32_t poll = 0;

  // Receive and process MSP protocol
  serialMSPreceive(1);

  if(millis() > poll + 500)     // this execute 2 times per second
  {
    poll = millis(); 

    // T1 and T2 special values
    setSensorT1andT2();

    // send smartport data
    telemetry.send();
  }
}


//------------------------------------------------------------------------
// FSSP_DATAID_T1         0x0400
// FSSP_DATAID_T2         0x0410
// FSSP_DATAID_RPM        0x050F
void setSensorT1andT2()
{
  uint16_t gT1, gT2 = 0;
  //
  // Calc T1
  //
  gT1 = 0;

  // ones column (lua modeE)
//    if (mode.armed) tmpi += 1;
//    else tmpi += 2;

  if (gMwSensorActive & mode.armed) gT1 += 4;

  // tens column (lua modeD)
  if (gMwSensorActive & mode.stable)  gT1 += 10;
  else if (gMwSensorActive & mode.horizon) gT1 += 20;
  else gT1 += 40; // acro

  // hundreds column (lua modeC)
  if (gMwSensorActive & mode.mag)     gT1 += 100;
  if (gMwSensorActive & mode.baro)    gT1 += 200;
  if (gMwSensorActive & mode.gpshold) gT1 += 400;

  // thousands column (lua modeB)
  if (gMwSensorActive & mode.gpshome) gT1 += 1000;
  if (gMwSensorActive & mode.cruise)  gT1 += 2000;
  if (gMwSensorActive & mode.gpsmission)  gT1 += 4000;
  // HEADFREE MODE tmpi += 8000;

  //ten thousands column (lua modeA)
  if (gMwSensorActive & mode.flaperon) gT1 += 10000;
  if (gMwSensorActive & mode.failsafe) gT1 += 20000;
  if (gMwSensorActive & mode.autotune) gT1 += 40000;


/* From iNAV CODE:    
    if (!isArmingDisabled())
        tmpi += 1;
    else
        tmpi += 2;
    if (ARMING_FLAG(ARMED))
        tmpi += 4;

    // tens column
    if (FLIGHT_MODE(ANGLE_MODE))
        tmpi += 10;
    if (FLIGHT_MODE(HORIZON_MODE))
        tmpi += 20;
    if (FLIGHT_MODE(MANUAL_MODE))
        tmpi += 40;

    // hundreds column
    if (FLIGHT_MODE(HEADING_MODE))
        tmpi += 100;
    if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
        tmpi += 200;
    if (FLIGHT_MODE(NAV_POSHOLD_MODE))
        tmpi += 400;

    // thousands column
    if (FLIGHT_MODE(NAV_RTH_MODE))
        tmpi += 1000;
    if (FLIGHT_MODE(NAV_CRUISE_MODE)) // intentionally out of order and 'else-ifs' to prevent column overflow
        tmpi += 8000;
    else if (FLIGHT_MODE(NAV_WP_MODE))
        tmpi += 2000;
    else if (FLIGHT_MODE(HEADFREE_MODE))
        tmpi += 4000;

    // ten thousands column
    if (FLIGHT_MODE(FLAPERON))
        tmpi += 10000;
    if (FLIGHT_MODE(FAILSAFE_MODE))
        tmpi += 40000;
    else if (FLIGHT_MODE(AUTO_TUNE)) // intentionally reverse order and 'else-if' to prevent 16-bit overflow
        tmpi += 20000;
*/

  //
  // Calc T2
  //
  // ones and tens columns (# of satellites 0 - 99)
  gT2 += constrain(gGPS_sats, 0, 99);

  // hundreds column (satellite accuracy HDOP: 0 = worst [HDOP > 5.5], 9 = best [HDOP <= 1.0])
  // From iNAV CODE:
  //tmpi += (9 - constrain((gGPS_hdop - 51) / 50, 0, 9)) * 100;

  /*uint16_t hdop = ((gGPS_hdop - 51) / 50);
  if (hdop < 0) hdop = 0;
  if (hdop > 9) hdop = 9;
  hdop = 9 - hdop * 100;
  */
  gT2 += (9 - constrain((gGPS_hdop - 51) / 50, 0, 9)) * 100;

  // thousands column (GPS fix status)
  if (gGPS_fix) gT2 += 1000;
  if (gGPS_homeFix) gT2 += 2000;

  // TODO:
  // from iNAV CODE: 
  //if (ARMING_FLAG(ARMED) && IS_RC_MODE_ACTIVE(BOXHOMERESET) && !FLIGHT_MODE(NAV_RTH_MODE) && !FLIGHT_MODE(NAV_WP_MODE))
  //  tmpi += 4000;

  // set sensor data. RPM no implemented, set RPM to 0
  rpm.setData(0, gT1, gT2);
}
