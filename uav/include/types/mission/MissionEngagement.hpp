#pragma once

#include "Position.hpp"

struct MissionEngagement {
  Position predPos;
  float horizonDistance;
  float distToPrediction;
  bool stagingMode;
};
