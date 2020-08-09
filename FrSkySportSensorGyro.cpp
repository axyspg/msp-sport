/*
  Gyro class for Teensy 3.x/4.0/LC and 328P, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 202000503
  Not for commercial use
*/

#include "FrSkySportSensorGyro.h" 

FrSkySportSensorGyro::FrSkySportSensorGyro(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorGyro::setData(float pitch, float roll, float accx, float accy, float accz)
{
  pitchData = (int16_t)(pitch);
  rollData = (int16_t)(roll);
  accxData = (int16_t)(accx);
  accyData = (int16_t)(accy);
  acczData = (int16_t)(accz);
}

uint16_t FrSkySportSensorGyro::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        dataId = GYRO_PITCH_DATA_ID;
        if(now > pitchTime)
        {
          pitchTime = now + GYRO_PITCH_DATA_PERIOD;
          serial.sendData(dataId, pitchData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 1:
        dataId = GYRO_ROLL_DATA_ID;
        if(now > rollTime)
        {
          rollTime = now + GYRO_ROLL_DATA_PERIOD;
          serial.sendData(dataId, rollData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 2:
        dataId = GYRO_ACCX_DATA_ID;
        if(now > accxTime)
        {
          accxTime = now + GYRO_ACCX_DATA_PERIOD;
          serial.sendData(dataId, accxData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 3:
        dataId = GYRO_ACCY_DATA_ID;
        if(now > accyTime)
        {
          accyTime = now + GYRO_ACCY_DATA_PERIOD;
          serial.sendData(dataId, accyData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 4:
        dataId = GYRO_ACCZ_DATA_ID;
        if(now > acczTime)
        {
          acczTime = now + GYRO_ACCZ_DATA_PERIOD;
          serial.sendData(dataId, acczData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= GYRO_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

/*uint16_t FrSkySportSensorGyro::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case RPM_T1_DATA_ID:
        t1 = (int32_t)data;
        return appId;
      case RPM_T2_DATA_ID:
        t2 = (int32_t)data;
        return appId;
      case RPM_ROT_DATA_ID:
        rpm = (uint32_t)(data / 2);
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

int32_t FrSkySportSensorGyro::getPitch() { return pitch; }
int32_t FrSkySportSensorGyro::getRoll() { return roll; }
int32_t FrSkySportSensorGyro::getAccX() { return accx; }
int32_t FrSkySportSensorGyro::getAccY() { return accy; }
int32_t FrSkySportSensorGyro::getAccZ() { return accz; }
*/
