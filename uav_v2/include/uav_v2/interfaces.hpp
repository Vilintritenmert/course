#pragma once

#include <string>

#include "uav_v2/types.hpp"

namespace uav
{

  class IConfigLoader
  {
  public:
    virtual void load(const std::string &dataDir) = 0;
    virtual auto getConfig() const -> const DroneConfig & = 0;
    virtual auto getAmmoParams() const -> const AmmoParams & = 0;
    virtual ~IConfigLoader() = default;
  };

  class ITargetProvider
  {
  public:
    virtual void load(const std::string &filePath) = 0;
    virtual auto getTargetCount() const -> int = 0;
    virtual auto getTarget(int index) const -> Target = 0;

    virtual void setArrayTimeStep(float dt) = 0;

    virtual void advance(float dt) = 0;

    virtual void reset() = 0;

    virtual ~ITargetProvider() = default;
  };

  class IBallisticSolver
  {
  public:
    virtual void init(const AmmoParams &ammo, float attackSpeed, float altitude,
                      float accelPath) = 0;
    virtual auto solve(const Coord &dronePos, const Target &target) const
        -> TargetCandidate = 0;

    virtual auto getHorizontalDistance() const -> float = 0;

    virtual ~IBallisticSolver() = default;
  };

}
