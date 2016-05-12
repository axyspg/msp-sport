/********************************************************************************************************
 * MSP to SmartPort Telemetry (simplified version)
 *
 * Original : https://github.com/axyspg/msp-sport 
 *            http://someonedidthatalready.blogspot.com  
 *  
 * MWOSD      Copyright (c) ShikOfTheRa (http://www.mwosd.com/)
 *                      GNU GPL 3 License http://www.gnu.org/licenses/
 *                      BSD License http://www.opensource.org/licenses/bsd-license.php
 *            
 * FrSkySport Library   (c) Pawelsky 20150725 (http://www.rcgroups.com/forums/showthread.php?t=2245978)
 *                      Not for commercial use
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

 
//#include <avr/pgmspace.h>
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
#include "GlobalVariables.h"

//#include "FrSkySportSensorRpm.h"
//FrskySportRpm rpm;
FrSkySportTelemetry telemetry;
FrSkySportSensorFcs vfas;
FrSkySportSensorGps gps;
FrSkySportSensorVario vario;

//------------------------------------------------------------------------
void setup()
{
  unsigned long elapsed = millis();

  // Initialize MSP to SmartPort port
  Serial.begin(BAUDRATE);
  Serial.flush();

  // Initialize SmartPort
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_2, &vfas, &gps, &vario);

}

//------------------------------------------------------------------------
void loop()
{
  if(millis() > timer.seconds+500)     // this execute 2 time a second
  {
    timer.seconds+=500;
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
  telemetry.send();  
}

