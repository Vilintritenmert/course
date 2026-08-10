
#include <cfloat>
#include <cmath>
#include <memory>

#include "DroneContext.hpp"
#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"
#include "StateStopped.hpp"

DroneContext::DroneContext(const std::shared_ptr<Config> config,
                           const std::shared_ptr<AmmoConfig> ammoConfigObj)
    : _config(config), _ammoConfig(ammoConfigObj),
      _position(config->getStartPos()),
      _ammo(*ammoConfigObj->findAmmo(config->getAmmoName())),
      _currentState(std::make_unique<StateStopped>()),
      _direction(config->getInitialDir()), _accelPath(config->getAccelPath()) {
  validateParameters();
}

DroneContext::~DroneContext() = default;

DroneState DroneContext::getState() const { return _currentState->id(); }

float DroneContext::normalizeAngle(float angle) const{
  while (angle > (float)M_PI)
    angle -= 2.0f * (float)M_PI;
  while (angle < -(float)M_PI)
    angle += 2.0f * (float)M_PI;
  return angle;
}

void DroneContext::reset() {
  _position = _config->getStartPos();
  _direction = _config->getInitialDir();
  _speed = 0.0f;
  _currentState = std::make_unique<StateStopped>();
}

int DroneContext::selectTarget(ITargetProvider* targetProvider,
                               int currentTargetIdx, float turnTimeLeft) {
  float timeToStop = 0.0f;
  switch (getState()) {
  case MOVING:
    timeToStop = getConfig()->getAttackSpeed() / getAcceleration();
    break;
  case ACCELERATING:
  case DECELERATING:
    timeToStop = getSpeed() / getAcceleration();
    break;
  case TURNING:
    timeToStop = turnTimeLeft;
    break;
  default:
    timeToStop = 0.0f;
  }

  int best = -1;
  float bestTime = FLT_MAX;

  for (int i = 0; i < targetProvider->getTargetCount(); ++i) {
    Position targetPos = targetProvider->getTarget(i);
    float tTime = Position::distance(targetPos - _position) /
                  std::max(getConfig()->getAttackSpeed(), 1.0e-6f);

    float effective = tTime + (i != currentTargetIdx ? timeToStop : 0.0f);
    if (effective < bestTime) {
      bestTime = effective;
      best = i;
    }
  }

  return best;
}

float DroneContext::getAcceleration() const {
  return getConfig()->getAttackSpeed() * getConfig()->getAttackSpeed() / (2.0f * getAccelPath());
};

float DroneContext::getAngleDelta() const {
  return normalizeAngle(getDisiredDirection() - getDirection());
}

void DroneContext::updateDrone(float desiredDir, float &turnAngleLeft) {
  setDisiredDirection(desiredDir);
  setTurnRemaining(turnAngleLeft);

  auto next = _currentState->execute(*this);
  if (next) {
    _currentState = std::move(next);
  }

  turnAngleLeft = getTurnRemaining();
}

void DroneContext::setSpeed(float speed) { _speed = speed; };

void DroneContext::setDirection(float direction) {
  _direction = normalizeAngle(direction);
};

void DroneContext::setPosition(Position position) { _position = position; };

void DroneContext::setTurnRemaining(float turnRemaining) {
  _turnRemaining = turnRemaining;
};

void DroneContext::setTargetDirection(float targetDirection) {
  _targetDirection = targetDirection;
};


void DroneContext::validateParameters() const {
  if (getConfig()->getAttackSpeed() <= 0.0f || getAccelPath() <= 0.0f ||
      getConfig()->getAltitude() <= 0.0f) {
    throw std::runtime_error("ERROR: Invalid parameters\n");
  }
}
