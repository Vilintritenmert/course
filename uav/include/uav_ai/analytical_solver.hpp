#pragma once

#include "uav_ai/interfaces.hpp"

namespace uav {

// Аналітичний розв'язок балістики (метод Кардано, формула з ДЗ1) з
// урахуванням lead targeting: передбачає, де буде ціль на момент падіння
// боєприпаса, і повертає точку скиду.
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
