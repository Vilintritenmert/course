#pragma once

#include "DroneContext.hpp"
#include "Position.hpp"

class IBallisticSolver {
public:
  virtual float computeFlightTime(std::shared_ptr<DroneContext> droneDetails) = 0;
  virtual float computeHorizDist(float t, std::shared_ptr<DroneContext> droneDetails) = 0;
  virtual bool computeDropPoint(Position tgt, Position drone, float h,
                                Position &drop) = 0;
  virtual ~IBallisticSolver() = default;
};