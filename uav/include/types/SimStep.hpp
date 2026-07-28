#pragma once

#include "Position.hpp"
#include <cfloat>
#include <cmath>

class SimStep {
public:
  Position pos;
  float direction;
  int state, targetIdx;

  Position dropPoint, aimPoint, predictedTarget;
};
