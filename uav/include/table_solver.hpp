#pragma once

#include <string>

#include "ballistic_table.hpp"
#include "interfaces.hpp"

namespace uav {

class TableSolver : public IBallisticSolver {
private:
  std::string tablePath_;
  BallisticTable table_;
  bool tableLoaded_ = false;

  float attackSpeed_ = 0.F;
  float accelPath_ = 0.F;
  float acceleration_ = 0.F;
  float fallTime_ = 0.F;
  float horizontalDistance_ = 0.F;
  bool ready_ = false;

public:
  static constexpr const char *kDefaultTablePath = "data/ballistic_table.txt";

  explicit TableSolver(std::string tablePath = kDefaultTablePath);

  auto init(const AmmoParams &ammo, float attackSpeed, float altitude,
            float accelPath) -> bool override;
  auto solve(const Coord &dronePos,
             const Target &target) const -> TargetCandidate override;
  auto getHorizontalDistance() const -> float override {
    return horizontalDistance_;
  }
};

} // namespace uav
