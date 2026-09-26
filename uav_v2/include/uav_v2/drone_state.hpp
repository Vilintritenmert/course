#pragma once

#include <memory>

#include "uav_v2/types.hpp"

namespace uav {

struct DroneContext {
  Coord &pos;
  float &direction;
  float &speed;
  float &targetDir;
  float &turnRemaining;
  float &turnSign;

  float desiredDir = 0.F;
  float acceleration = 0.F;
  const DroneConfig &cfg;
};

class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> = 0;

  virtual auto timeToStop(const DroneContext &ctx) const -> float {
    (void)ctx;
    return 0.F;
  }

  virtual auto name() const -> const char * = 0;
};

class StateStopped : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto name() const -> const char * override { return "Stopped"; }
};

class StateAccelerating : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Accelerating"; }
};

class StateDecelerating : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Decelerating"; }
};

class StateTurning : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Turning"; }
};

class StateMoving : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Moving"; }
};

}
