#include "types.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include "target_provider.hpp"

int DroneDetails::selectTarget(ITargetProvider *targetProvider,
                               TimeManagement *timeManager, float zd,
                               int targetCount, int currentTargetIdx,
                               float turnTimeLeft, Position outDrop[],
                               Position outPred[]) {
  (void)timeManager;
  (void)zd;

  float timeToStop = 0.0f;
  switch (_state) {
  case ACCELERATING:
    timeToStop = _speed / getAcceleration();
    break;
  case MOVING:
    timeToStop = _attackSpeed / getAcceleration();
    break;
  case DECELERATING:
    timeToStop = _speed / getAcceleration();
    break;
  case TURNING:
    timeToStop = turnTimeLeft;
    break;
  default:
    timeToStop = 0.0f;
  }

  int best = -1;
  float bestTime = FLT_MAX;
  for (int i = 0; i < targetCount; ++i) {
    Position targetPos = targetProvider->getTarget(i);
    Position dPos = targetPos;
    Position pPos = targetPos;
    float tTime = Position::distance(targetPos - _position) /
                  std::max(_attackSpeed, 1.0e-6f);

    outDrop[i] = dPos;
    outPred[i] = pPos;

    float effective = tTime + (i != currentTargetIdx ? timeToStop : 0.0f);
    if (effective < bestTime) {
      bestTime = effective;
      best = i;
    }
  }

  return best;
}

void DroneDetails::updateDrone(Position &pos, float &dir, float &speed,
                               DroneState &state, float desiredDir, float dt,
                               float attackSpeed, float accel,
                               float angularSpeed, float turnThreshold,
                               float &turnAngleLeft) {
  float angleDiff = normalizeAngle(desiredDir - dir);
  float angStep = angularSpeed * dt;

  switch (state) {
  case STOPPED:
    speed = 0.0f;
    if (fabsf(angleDiff) > turnThreshold) {
      state = TURNING;
      turnAngleLeft = fabsf(angleDiff);
    } else {
      state = ACCELERATING;
    }
    break;

  case ACCELERATING:
    if (fabsf(angleDiff) > turnThreshold) {
      state = DECELERATING;
    } else {
      dir += (fabsf(angleDiff) <= angStep)
                 ? angleDiff
                 : (angleDiff > 0 ? angStep : -angStep);
      dir = normalizeAngle(dir);
      speed += accel * dt;
      if (speed >= attackSpeed) {
        speed = attackSpeed;
        state = MOVING;
      }
    }
    pos = pos + Position{cosf(dir), sinf(dir)} * (speed * dt);
    break;

  case MOVING:
    if (fabsf(angleDiff) > turnThreshold) {
      state = DECELERATING;
    } else {
      dir += (fabsf(angleDiff) <= angStep)
                 ? angleDiff
                 : (angleDiff > 0 ? angStep : -angStep);
      dir = normalizeAngle(dir);
    }
    pos = pos + Position{cosf(dir), sinf(dir)} * (speed * dt);
    break;

  case DECELERATING:
    speed -= accel * dt;
    if (speed <= 0.0f) {
      speed = 0.0f;
      state = TURNING;
      turnAngleLeft = fabsf(angleDiff);
    } else {
      pos = pos + Position{cosf(dir), sinf(dir)} * (speed * dt);
    }
    break;

  case TURNING:
    if (fabsf(angleDiff) <= angStep) {
      dir = desiredDir;
      state = ACCELERATING;
      turnAngleLeft = 0.0f;
    } else {
      dir += (angleDiff > 0 ? angStep : -angStep);
      dir = normalizeAngle(dir);
      turnAngleLeft = fabsf(angleDiff) - angStep;
      if (turnAngleLeft < 0.0f)
        turnAngleLeft = 0.0f;
    }
    break;
  }

  _position = pos;
  _direction = dir;
  _speed = speed;
  _state = state;
}

