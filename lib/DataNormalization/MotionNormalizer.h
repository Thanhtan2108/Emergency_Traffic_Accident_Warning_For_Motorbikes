#pragma once
#include "MotionSample.h"
#include "NormalizedMotionData.h"

class MotionNormalizer {
public:
  MotionNormalizer();

  NormalizedMotionData normalize(const MotionSample& raw);

private:
  float vectorMagnitude(float x, float y, float z);
};
