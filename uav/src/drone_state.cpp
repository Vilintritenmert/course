#include "uav_ai/drone_state.hpp"

#include <algorithm>
#include <cmath>

namespace uav {

namespace {

// Спільна логіка для STOPPED / ACCELERATING / MOVING: якщо курс на ціль
// відхиляється сильно - почати гальмування (щоб потім розвернутись на
// місці), інакше довернути курс і або розганятись, або йти на
// крейсерській швидкості.
auto advanceTowardTarget(DroneContext &ctx) -> std::unique_ptr<IDroneState> {
  const float delta = normalizeAngle(ctx.desiredDir - ctx.direction);

  if (std::fabs(delta) > ctx.cfg.turnThreshold) {
    ctx.targetDir = ctx.desiredDir;
    if (ctx.speed > 0.F) {
      return std::make_unique<StateDecelerating>();
    }
    ctx.turnSign = (delta >= 0.F) ? 1.F : -1.F;
    ctx.turnRemaining = std::fabs(delta) / ctx.cfg.angularSpeed;
    return std::make_unique<StateTurning>();
  }

  ctx.direction = ctx.desiredDir;
  const float dt = ctx.cfg.simTimeStep;
  const Coord dir = {std::cos(ctx.direction), std::sin(ctx.direction)};

  if (ctx.speed < ctx.cfg.attackSpeed) {
    ctx.speed = std::min(ctx.cfg.attackSpeed, ctx.speed + ctx.acceleration * dt);
    ctx.pos = ctx.pos + dir * (ctx.speed * dt);
    if (ctx.speed < ctx.cfg.attackSpeed) {
      return std::make_unique<StateAccelerating>();
    }
    return std::make_unique<StateMoving>();
  }

  ctx.speed = ctx.cfg.attackSpeed;
  ctx.pos = ctx.pos + dir * (ctx.speed * dt);
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
  const float dt = ctx.cfg.simTimeStep;
  ctx.speed -= ctx.acceleration * dt;
  const Coord dir = {std::cos(ctx.direction), std::sin(ctx.direction)};
  ctx.pos = ctx.pos + dir * (std::max(0.F, ctx.speed) * dt);

  if (ctx.speed <= 0.F) {
    ctx.speed = 0.F;
    return std::make_unique<StateStopped>();
  }
  return nullptr;
}

auto StateDecelerating::timeToStop(const DroneContext &ctx) const -> float {
  return ctx.speed / ctx.acceleration;
}

auto StateTurning::execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> {
  const float dt = ctx.cfg.simTimeStep;
  if (ctx.turnRemaining <= dt) {
    ctx.direction = ctx.targetDir;
    ctx.turnRemaining = 0.F;
    return std::make_unique<StateAccelerating>();
  }
  ctx.direction =
      normalizeAngle(ctx.direction + ctx.turnSign * ctx.cfg.angularSpeed * dt);
  ctx.turnRemaining -= dt;
  return nullptr;
}

auto StateTurning::timeToStop(const DroneContext &ctx) const -> float {
  return ctx.turnRemaining;
}

} // namespace uav
