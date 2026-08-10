#include <cmath>

#include "DroneContext.hpp"
#include "StateAccelerating.hpp"
#include "StateStopped.hpp"
#include "StateTurning.hpp"

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext &ctx) {
  float angleDiff =
      ctx.normalizeAngle(ctx.getDisiredDirection() - ctx.getDirection());

  ctx.setSpeed(0.0f);

  if (fabsf(angleDiff) > ctx.getConfig()->getTurnThreshold()) {
    ctx.setTurnRemaining(fabsf(angleDiff));
    return std::make_unique<StateTurning>();
  }

  return std::make_unique<StateAccelerating>();
}
