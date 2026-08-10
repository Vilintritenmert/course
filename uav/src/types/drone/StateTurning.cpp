#include <cmath>

#include "DroneContext.hpp"
#include "StateAccelerating.hpp"
#include "StateTurning.hpp"

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext &ctx) {
  float angleDiff =
      ctx.normalizeAngle(ctx.getDisiredDirection() - ctx.getDirection());
  float angStep =
      ctx.getConfig()->getAngularSpeed() * ctx.getConfig()->getSimTimeStep();

  if (fabsf(angleDiff) <= angStep) {
    ctx.setDirection(ctx.getDisiredDirection());
    ctx.setTurnRemaining(0.0f);
    return std::make_unique<StateAccelerating>();
  }

  ctx.setDirection(ctx.getDirection() + (angleDiff > 0 ? angStep : -angStep));
  float turnRemaining = fabsf(angleDiff) - angStep;
  if (turnRemaining < 0.0f) {
    turnRemaining = 0.0f;
  }
  ctx.setTurnRemaining(turnRemaining);

  return nullptr;
}
