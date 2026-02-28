#include "MotionSensor.h"

MotionSensor::MotionSensor(IMUDriver* driver)
  : _driver(driver)
{
  _lastSample = {0, 0, 0, 0, 0, 0, 0};
}

bool MotionSensor::begin() {
  if (!_driver) return false;
  return _driver->begin();
}

void MotionSensor::calibrateGyro(bool blocking) {
  if (_driver) {
    _driver->calibrateGyro(blocking);
  }
}

MotionSample MotionSensor::readRaw() {
  _driver->update();

  MotionSample s;
  s.ax = _driver->getAccX() * G_TO_MSS;
  s.ay = _driver->getAccY() * G_TO_MSS;
  s.az = _driver->getAccZ() * G_TO_MSS;

  s.gx = _driver->getGyroX();
  s.gy = _driver->getGyroY();
  s.gz = _driver->getGyroZ();

  s.timestamp = millis();

  _lastSample = s;
  return s;
}

MotionSample MotionSensor::last() const {
  return _lastSample;
}
