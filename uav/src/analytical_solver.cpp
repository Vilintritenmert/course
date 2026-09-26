#include "analytical_solver.hpp"

#include <algorithm>
#include <cmath>

namespace uav {

namespace {

auto computeTimeOfFlight(const AmmoParams &ammo, double attackSpeed,
                         double dropHeight, double &outTime) -> bool {
  const double m = ammo.mass;
  const double d = ammo.drag;
  const double l = ammo.lift;
  const double v0 = attackSpeed;
  const double z0 = dropHeight;

  const double a = d * GRAVITY_ACCELERATION * m - 2. * d * d * l * v0;
  const double b = -3. * GRAVITY_ACCELERATION * m * m + 3. * d * l * m * v0;
  const double c = 6. * m * m * z0;

  if (std::fabs(a) < EPSILON) {
    return false;
  }

  const double p = -(b * b) / (3. * a * a);
  const double q = (2. * b * b * b) / (27. * a * a * a) + c / a;

  if (p >= 0.) {
    return false;
  }

  const double acosArg = (3. * q) / (2. * p) * std::sqrt(-3. / p);
  if (acosArg < -1. || acosArg > 1.) {
    return false;
  }

  const double phi = std::acos(acosArg);
  const double t =
      2. * std::sqrt(-p / 3.) * std::cos((phi + 4. * M_PI) / 3.) - b / (3. * a);

  if (t <= 0.) {
    return false;
  }

  outTime = t;
  return true;
}

auto computeHorizontalDistance(const AmmoParams &ammo, double attackSpeed,
                               double t) -> double {
  const double m = ammo.mass;
  const double d = ammo.drag;
  const double l = ammo.lift;
  const double v0 = attackSpeed;

  const double term1 = v0 * t;
  const double term2 = (t * t * d * v0) / (2. * m);
  const double term3 =
      (std::pow(t, 3) * (6. * d * GRAVITY_ACCELERATION * l * m -
                         6. * d * d * (l * l - 1.) * v0)) /
      (36. * m * m);
  const double term4 =
      (std::pow(t, 4) *
       (-6. * d * d * GRAVITY_ACCELERATION * l * (1. + l * l + std::pow(l, 4)) *
            m +
        3. * std::pow(d, 3) * l * l * (1. + l * l) * v0 +
        6. * std::pow(d, 3) * std::pow(l, 4) * (1. + l * l) * v0)) /
      (36. * std::pow(1. + l * l, 2) * std::pow(m, 3));
  const double term5 =
      (std::pow(t, 5) *
       (3. * std::pow(d, 3) * GRAVITY_ACCELERATION * std::pow(l, 3) * m -
        3. * std::pow(d, 4) * l * l * (1. + l * l) * v0)) /
      (36. * (1. + l * l) * std::pow(m, 4));

  return term1 - term2 + term3 + term4 + term5;
}

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

auto AnalyticalSolver::init(const AmmoParams &ammo, float attackSpeed,
                            float altitude, float accelPath) -> bool {
  ready_ = false;
  ammo_ = ammo;
  attackSpeed_ = attackSpeed;
  accelPath_ = accelPath;

  if (!computeTimeOfFlight(ammo_, attackSpeed_, altitude, fallTime_)) {
    return false;
  }
  horizontalDistance_ = static_cast<float>(
      computeHorizontalDistance(ammo_, attackSpeed_, fallTime_));
  acceleration_ = (attackSpeed_ * attackSpeed_) / (2.F * accelPath_);

  ready_ = true;
  return true;
}

auto AnalyticalSolver::solve(const Coord &dronePos, const Target &target) const
    -> TargetCandidate {
  TargetCandidate candidate;
  if (!ready_) {
    return candidate;
  }

  const float distNow = length(target.pos - dronePos);
  const float roughReleaseDist = std::max(0.F, distNow - horizontalDistance_);
  const float orientTotalTime = estimateTravelTime(
      roughReleaseDist, attackSpeed_, acceleration_, accelPath_);

  const Coord predicted =
      target.pos +
      target.velocity * (orientTotalTime + static_cast<float>(fallTime_));

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
