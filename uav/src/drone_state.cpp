#include "drone_state.hpp"

#include <cmath>

namespace uav {

namespace {

auto advanceTowardTarget(DroneContext &ctx) -> std::unique_ptr<IDroneState> {
  const float delta = normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(delta) > ctx.cfg.turnThreshold) {
    if (ctx.speed > 0.F) {
      ctx.command = {DroneMode::Decelerating, 0.F};
      return std::make_unique<StateDecelerating>();
    }
    const float sign = (delta >= 0.F) ? 1.F : -1.F;
    ctx.command = {DroneMode::Turning, sign * ctx.cfg.angularSpeed};
    return std::make_unique<StateTurning>();
  }

  if (ctx.speed < ctx.cfg.attackSpeed) {
    ctx.command = {DroneMode::Accelerating, 0.F};
    return std::make_unique<StateAccelerating>();
  }

  ctx.command = {DroneMode::Moving, 0.F};
  return std::make_unique<StateMoving>();
}

} // namespace

auto StateStopped::execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> {
  return advanceTowardTarget(ctx);
}

auto StateAccelerating::execute(DroneContext &ctx)
    -> std::unique_ptr<IDroneState> {
  return advanceTowardTarget(ctx);
}

auto StateAccelerating::timeToStop(const DroneContext &ctx) const -> float {
  return ctx.speed / ctx.acceleration;
}

auto StateMoving::execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> {
  return advanceTowardTarget(ctx);
}

auto StateMoving::timeToStop(const DroneContext &ctx) const -> float {
  return ctx.speed / ctx.acceleration;
}

auto StateDecelerating::execute(DroneContext &ctx)
    -> std::unique_ptr<IDroneState> {
  if (ctx.speed <= 0.F) {
    ctx.command = {DroneMode::Stopped, 0.F};
    return std::make_unique<StateStopped>();
  }
  ctx.command = {DroneMode::Decelerating, 0.F};
  return nullptr;
}

auto StateDecelerating::timeToStop(const DroneContext &ctx) const -> float {
  return ctx.speed / ctx.acceleration;
}

auto StateTurning::execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> {
  const float delta = normalizeAngle(ctx.desiredDir - ctx.direction);
  if (std::fabs(delta) <= ctx.cfg.turnThreshold) {
    ctx.command = {DroneMode::Accelerating, 0.F};
    return std::make_unique<StateAccelerating>();
  }
  const float sign = (delta >= 0.F) ? 1.F : -1.F;
  ctx.command = {DroneMode::Turning, sign * ctx.cfg.angularSpeed};
  return nullptr;
}

auto StateTurning::timeToStop(const DroneContext &ctx) const -> float {
  const float delta = std::fabs(normalizeAngle(ctx.desiredDir - ctx.direction));
  return delta / ctx.cfg.angularSpeed;
}

} // namespace uav
