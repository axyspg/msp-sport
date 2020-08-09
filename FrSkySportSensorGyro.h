/*
  FrSky GYRO sensor class for Teensy 3.x/4.0/LC, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 202000503 (not official from Pawelsky)
  Not for commercial use

  decode not implemented
*/

#ifndef _FRSKY_SPORT_SENSOR_GYRO_H_
#define _FRSKY_SPORT_SENSOR_GYRO_H_

#include "FrSkySportSensor.h"

#define GYRO_DEFAULT_ID ID11
#define GYRO_DATA_COUNT 5

#define GYRO_PITCH_DATA_ID      0x0430
#define GYRO_ROLL_DATA_ID       0x0440
#define GYRO_ACCX_DATA_ID       0x0700
#define GYRO_ACCY_DATA_ID       0x0710
#define GYRO_ACCZ_DATA_ID       0x0720

#define GYRO_PITCH_DATA_PERIOD  100
#define GYRO_ROLL_DATA_PERIOD   100
#define GYRO_ACCX_DATA_PERIOD   100
#define GYRO_ACCY_DATA_PERIOD   100
#define GYRO_ACCZ_DATA_PERIOD   100

class FrSkySportSensorGyro : public FrSkySportSensor
{
  public:
    FrSkySportSensorGyro(SensorId id = GYRO_DEFAULT_ID);
    void setData(float pitch = 0.0, float roll = 0.0, float accx = 0.0, float accy = 0.0, float accz = 0.0);
    virtual uint16_t send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
/*    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    int32_t getPitch();
    int32_t getRoll();
    int32_t getAccX();
    int32_t getAccY();
    int32_t getAccZ();*/

  private:
    int16_t pitchData;
    int16_t rollData;
    int16_t accxData;
    int16_t accyData;
    int16_t acczData;
    uint32_t pitchTime;
    uint32_t rollTime;
    uint32_t accxTime;
    uint32_t accyTime;
    uint32_t acczTime;
/*    int32_t pitch;
    int32_t roll;
    int32_t accx;
    int32_t accy;
    int32_t accz;*/
};

#endif // _FRSKY_SPORT_SENSOR_GYRO_H_
