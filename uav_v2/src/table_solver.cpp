#include "uav_v2/table_solver.hpp"

#include "uav_v2/uav_exception.hpp"

#include <algorithm>
#include <cmath>

namespace uav {

  namespace {

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

  TableSolver::TableSolver(std::string tablePath) : tablePath_(std::move(tablePath)) {
  }

  void TableSolver::init(const AmmoParams &ammo, float attackSpeed, float altitude,
                         float accelPath) {
    ready_ = false;

    if (!tableLoaded_) {
      table_.load(tablePath_);
      tableLoaded_ = true;
    }

    attackSpeed_ = attackSpeed;
    accelPath_ = accelPath;

    const BallisticTable::Result r =
        table_.lookup(altitude, attackSpeed_, ammo.mass, ammo.drag, ammo.lift);
    fallTime_ = r.t;
    horizontalDistance_ = r.hDist;
    acceleration_ = (attackSpeed_ * attackSpeed_) / (2.F * accelPath_);

    ready_ = fallTime_ > 0.F;
    if (!ready_) {
      throw UavException("Error: failed to initialize table solver");
    }
  }

  auto TableSolver::solve(const Coord &dronePos, const Target &target) const -> TargetCandidate {
    TargetCandidate candidate;
    if (!ready_) {
      throw UavException("Error: table solver is not initialized");
    }

    const float distNow = length(target.position - dronePos);
    const float roughReleaseDist = std::max(0.F, distNow - horizontalDistance_);
    const float orientTotalTime =
        estimateTravelTime(roughReleaseDist, attackSpeed_, acceleration_, accelPath_);

    const Coord predicted = target.position + target.velocity * (orientTotalTime + fallTime_);

    const Coord delta = predicted - dronePos;
    const float distToPredicted = length(delta);
    const float heading = std::atan2(delta.y, delta.x);
    const float releaseDist = std::max(0.F, distToPredicted - horizontalDistance_);

    candidate.valid = true;
    candidate.heading = heading;
    candidate.releasePoint = predicted - normalize(delta) * releaseDist;
    candidate.totalTime = estimateTravelTime(releaseDist, attackSpeed_, acceleration_, accelPath_);
    candidate.predictedTarget = predicted;

    return candidate;
  }

} // namespace uav
