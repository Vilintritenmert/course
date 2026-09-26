#pragma once

#include <string>

#include "types.hpp"

namespace uav {

class IConfigLoader {
public:
  virtual auto load(const std::string &dataDir) -> bool = 0;
  virtual auto getConfig() const -> const DroneConfig & = 0;
  virtual auto getAmmoParams() const -> const AmmoParams & = 0;
  virtual ~IConfigLoader() = default;
};

class ITargetProvider {
public:
  virtual auto load(const std::string &filePath) -> bool = 0;
  virtual auto getTargetCount() const -> int = 0;
  virtual auto getTarget(int index) const -> Target = 0;

  virtual void setArrayTimeStep(float dt) = 0;

  virtual void advance(float dt) = 0;
  virtual void reset() = 0;

  virtual ~ITargetProvider() = default;
};

class IBallisticSolver {
public:
  virtual auto init(const AmmoParams &ammo, float attackSpeed, float altitude,
                     float accelPath) -> bool = 0;
  virtual auto solve(const Coord &dronePos, const Target &target) const
      -> TargetCandidate = 0;

  virtual auto getHorizontalDistance() const -> float = 0;

  virtual ~IBallisticSolver() = default;
};

} // namespace uav
