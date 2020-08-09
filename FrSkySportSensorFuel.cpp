/*
  Fuel class for Teensy 3.x/4.0/LC and 328P, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 202000503
  Not for commercial use
*/

#include "FrSkySportSensorFuel.h" 

FrSkySportSensorFuel::FrSkySportSensorFuel(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorFuel::setData(uint8_t remain)
{
  remainData = (uint8_t)(remain);
}

uint16_t FrSkySportSensorFuel::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        dataId = FUEL_REMAIN_DATA_ID;
        if(now > remainTime)
        {
          remainTime = now + FUEL_REMAIN_DATA_PERIOD;
          serial.sendData(dataId, remainData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= FUEL_DATA_COUNT) sensorDataIdx = 0;
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
