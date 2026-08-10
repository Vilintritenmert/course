#include <cmath>

#include <memory>
#include "DroneContext.hpp"
#include "StateAccelerating.hpp"
#include "StateDecelerating.hpp"
#include "StateMoving.hpp"

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext &ctx) {
  float angleDiff =
      ctx.normalizeAngle(ctx.getDisiredDirection() - ctx.getDirection());
  float angStep =
      ctx.getConfig()->getAngularSpeed() * ctx.getConfig()->getSimTimeStep();

  std::unique_ptr<IDroneState> next = nullptr;

  if (fabsf(angleDiff) > ctx.getConfig()->getTurnThreshold()) {
    next = std::make_unique<StateDecelerating>();
  } else {
    ctx.setDirection(ctx.getDirection() +
                     (fabsf(angleDiff) <= angStep
                          ? angleDiff
                          : (angleDiff > 0 ? angStep : -angStep)));
    ctx.setSpeed(ctx.getSpeed() +
                 ctx.getAcceleration() * ctx.getConfig()->getSimTimeStep());
    if (ctx.getSpeed() >= ctx.getConfig()->getAttackSpeed()) {
      ctx.setSpeed(ctx.getConfig()->getAttackSpeed());
      next = std::make_unique<StateMoving>();
    }
  }

  ctx.setPosition(ctx.getPosition() +
                  Position{cosf(ctx.getDirection()), sinf(ctx.getDirection())} *
                      (ctx.getSpeed() * ctx.getConfig()->getSimTimeStep()));

  return next;
}
