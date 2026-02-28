#include "MotionNormalizer.h"
#include <math.h>

// DEG_TO_RAD = 0.017453292519943295f; defined into arduino.h

MotionNormalizer::MotionNormalizer() {}

NormalizedMotionData MotionNormalizer::normalize(const MotionSample& raw) {
  NormalizedMotionData data;

  data.ax = raw.ax;
  data.ay = raw.ay;
  data.az = raw.az;

  data.wx = raw.gx * DEG_TO_RAD;
  data.wy = raw.gy * DEG_TO_RAD;
  data.wz = raw.gz * DEG_TO_RAD;

  data.accMagnitude  = vectorMagnitude(data.ax, data.ay, data.az);
  data.gyroMagnitude = vectorMagnitude(data.wx, data.wy, data.wz);

  data.timestamp = raw.timestamp;
  return data;
}

float MotionNormalizer::vectorMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}
