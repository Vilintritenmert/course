#pragma once

#include <string>

#include "BallisticTable.hpp"
#include "IBallisticSolver.hpp"

class TableSolver : public IBallisticSolver {
private:
  BallisticTable _table;

public:
  explicit TableSolver(const std::string &tablePath = "data/ballistic_table.txt");

  float computeFlightTime(std::shared_ptr<DroneContext> droneDetails);

  float computeHorizDist(float t, std::shared_ptr<DroneContext> droneDetails);

  bool computeDropPoint(Position tgt, Position drone, float h, Position &drop);
};
