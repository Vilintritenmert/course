#pragma once

#include "interfaces.hpp"

namespace uav {

class AnalyticalSolver : public IBallisticSolver {
public:
  auto init(const AmmoParams &ammo, float attackSpeed, float altitude,
            float accelPath) -> bool override;
  auto solve(const Coord &dronePos, const Target &target) const
      -> TargetCandidate override;
  auto getHorizontalDistance() const -> float override {
    return horizontalDistance_;
  }

private:
  AmmoParams ammo_;
  float attackSpeed_ = 0.F;
  float accelPath_ = 0.F;
  float acceleration_ = 0.F;
  double fallTime_ = 0.;
  float horizontalDistance_ = 0.F;
  bool ready_ = false;
};

} // namespace uav
