/*
  FrSky FUEL sensor class for Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 202000503 (not official from Pawelsky)
  Not for commercial use

  decode not implemented
*/

#ifndef _FRSKY_SPORT_SENSOR_FUEL_H_
#define _FRSKY_SPORT_SENSOR_FUEL_H_

#include "FrSkySportSensor.h"

#define FUEL_DEFAULT_ID ID12
#define FUEL_DATA_COUNT 1

#define FUEL_REMAIN_DATA_ID      0x0600

#define FUEL_REMAIN_DATA_PERIOD  2000

class FrSkySportSensorFuel : public FrSkySportSensor
{
  public:
    FrSkySportSensorFuel(SensorId id = FUEL_DEFAULT_ID);
    void setData(uint8_t remain = 0);
    virtual uint16_t send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
/*    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    uint_8t getRemain();
    int32_t getAccZ();*/

  private:
    uint8_t remainData;
    uint32_t remainTime;
/*    int32_t pitch;
    int32_t roll;
    int32_t accx;
    int32_t accy;
    int32_t accz;*/
};

#endif // _FRSKY_SPORT_SENSOR_GYRO_H_
