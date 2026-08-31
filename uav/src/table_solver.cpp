#include "uav_ai/table_solver.hpp"

#include <algorithm>
#include <cmath>

namespace uav {

namespace {

// Орієнтовний час прольоту дистанції з розгоном/крейсерською швидкістю
// (той самий трапецієподібний профіль, що й в AnalyticalSolver).
auto estimateTravelTime(float distance, float cruiseSpeed, float acceleration,
                        float accelerationPath) -> float {
  if (distance <= 0.F) {
    return 0.F;
  }

  if (distance >= 2.F * accelerationPath) {
    const float accelTime = cruiseSpeed / acceleration;
    const float cruiseDist = distance - 2.F * accelerationPath;
    return 2.F * accelTime + cruiseDist / cruiseSpeed;
  }

  const float peakSpeed = std::sqrt(acceleration * distance);
  return 2.F * peakSpeed / acceleration;
}

} // namespace

TableSolver::TableSolver(std::string tablePath) : tablePath_(std::move(tablePath)) {}

auto TableSolver::init(const AmmoParams &ammo, float attackSpeed,
                       float altitude, float accelPath) -> bool {
  ready_ = false;

  if (!tableLoaded_) {
    tableLoaded_ = table_.load(tablePath_);
    if (!tableLoaded_) {
      return false;
    }
  }

  attackSpeed_ = attackSpeed;
  accelPath_ = accelPath;

  const BallisticTable::Result r =
      table_.lookup(altitude, attackSpeed_, ammo.mass, ammo.drag, ammo.lift);
  fallTime_ = r.t;
  horizontalDistance_ = r.hDist;
  acceleration_ = (attackSpeed_ * attackSpeed_) / (2.F * accelPath_);

  ready_ = fallTime_ > 0.F;
  return ready_;
}

auto TableSolver::solve(const Coord &dronePos, const Target &target) const
    -> TargetCandidate {
  TargetCandidate candidate;
  if (!ready_) {
    return candidate;
  }

  const float distNow = length(target.position - dronePos);
  const float roughReleaseDist = std::max(0.F, distNow - horizontalDistance_);
  const float orientTotalTime = estimateTravelTime(
      roughReleaseDist, attackSpeed_, acceleration_, accelPath_);

  const Coord predicted =
      target.position + target.velocity * (orientTotalTime + fallTime_);

  const Coord delta = predicted - dronePos;
  const float distToPredicted = length(delta);
  const float heading = std::atan2(delta.y, delta.x);
  const float releaseDist = std::max(0.F, distToPredicted - horizontalDistance_);

  candidate.valid = true;
  candidate.heading = heading;
  candidate.releasePoint = predicted - normalize(delta) * releaseDist;
  candidate.totalTime =
      estimateTravelTime(releaseDist, attackSpeed_, acceleration_, accelPath_);
  candidate.predictedTarget = predicted;

  return candidate;
}

} // namespace uav
