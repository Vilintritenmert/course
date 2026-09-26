#include <cmath>

#include "DroneContext.hpp"
#include "StateDecelerating.hpp"
#include "StateTurning.hpp"

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext &ctx) {
  float angleDiff =
      ctx.normalizeAngle(ctx.getDisiredDirection() - ctx.getDirection());

  ctx.setSpeed(ctx.getSpeed() -
               ctx.getAcceleration() * ctx.getConfig()->getSimTimeStep());

  if (ctx.getSpeed() <= 0.0f) {
    ctx.setSpeed(0.0f);
    ctx.setTurnRemaining(fabsf(angleDiff));
    return std::make_unique<StateTurning>();
  }

  ctx.setPosition(ctx.getPosition() +
                  Position{cosf(ctx.getDirection()), sinf(ctx.getDirection())} *
                      (ctx.getSpeed() * ctx.getConfig()->getSimTimeStep()));

  return nullptr;
}
