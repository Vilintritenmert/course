#pragma once

#include "Drone.hpp"
#include "Position.hpp"

class IBallisticSolver {
public:
  virtual float computeFlightTime(DroneDetails* droneDetails) = 0;
  virtual float computeHorizDist(float t, DroneDetails* droneDetails) = 0;
  virtual bool computeDropPoint(Position tgt, Position drone, float h,
                                Position &drop) = 0;
};