#pragma once

#include "IBallisticSolver.hpp"

const float G = 9.81f;

class AnalyticalSolver : public IBallisticSolver {
public:
  float computeFlightTime(DroneDetails *droneDetails);

  float computeHorizDist(float t, DroneDetails *droneDetails);

  bool computeDropPoint(Position tgt, Position drone, float h, Position &drop);
};
