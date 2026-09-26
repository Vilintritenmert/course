#include "Drone.hpp"

#include <cfloat>
#include <cmath>

#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"

DroneDetails::DroneDetails(const Config *config,
                           const AmmoConfig *ammoConfigObj)
    : _config(config), _ammoConfig(ammoConfigObj),
      _position(config->getStartPos()),
      _ammo(*ammoConfigObj->findAmmo(config->getAmmoName())),
      _altitude(config->getAltitude()), _direction(config->getInitialDir()),
      _attackSpeed(config->getAttackSpeed()),
      _angularSpeed(config->getAngularSpeed()),
      _accelPath(config->getAccelPath()) {
  validateParameters();
}

float DroneDetails::normalizeAngle(float a) {
  while (a > (float)M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}

void DroneDetails::reset() {
  _position = _config->getStartPos();
  _direction = _config->getInitialDir();
  _speed = 0.0f;
  _state = STOPPED;
}

int DroneDetails::selectTarget(ITargetProvider *targetProvider,
                               int currentTargetIdx, float turnTimeLeft) {
  float timeToStop = 0.0f;
  switch (getState()) {
  case MOVING:
    timeToStop = _attackSpeed / getAcceleration();
    break;
  case ACCELERATING:
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

  for (int i = 0; i < targetProvider->getTargetCount(); ++i) {
    Position targetPos = targetProvider->getTarget(i);
    float tTime = Position::distance(targetPos - _position) /
                  std::max(_attackSpeed, 1.0e-6f);

    float effective = tTime + (i != currentTargetIdx ? timeToStop : 0.0f);
    if (effective < bestTime) {
      bestTime = effective;
      best = i;
    }
  }

  return best;
}

float DroneDetails::getAcceleration() const {
  return _attackSpeed * _attackSpeed / (2.0f * _accelPath);
};

void DroneDetails::updateDrone(float desiredDir, float &turnAngleLeft) {
  float angleDiff = normalizeAngle(desiredDir - getDirection());
  float angStep = getAngularSpeed() * getConfig()->getSimTimeStep();

  switch (getState()) {
  case STOPPED:
    setSpeed(0.0f);
    if (fabsf(angleDiff) > getConfig()->getTurnThreshold()) {
      setState(TURNING);
      turnAngleLeft = fabsf(angleDiff);
    } else {
      setState(ACCELERATING);
    }
    break;

  case ACCELERATING:
    if (fabsf(angleDiff) > getConfig()->getTurnThreshold()) {
      setState(DECELERATING);
    } else {
      setDirection(getDirection() +
                   (fabsf(angleDiff) <= angStep
                        ? angleDiff
                        : (angleDiff > 0 ? angStep : -angStep)));
      setSpeed(getSpeed() + getAcceleration() * getConfig()->getSimTimeStep());
      if (getSpeed() >= getAttackSpeed()) {
        setSpeed(getAttackSpeed());
        setState(MOVING);
      }
    }
    setPosition(getPosition() +
                Position{cosf(getDirection()), sinf(getDirection())} *
                    (getSpeed() * getConfig()->getSimTimeStep()));
    break;

  case MOVING:
    if (fabsf(angleDiff) > getConfig()->getTurnThreshold()) {
      setState(DECELERATING);
    } else {
      setDirection(getDirection() +
                   (fabsf(angleDiff) <= angStep
                        ? angleDiff
                        : (angleDiff > 0 ? angStep : -angStep)));
      setSpeed(getSpeed() + getAcceleration() * getConfig()->getSimTimeStep());
      if (getSpeed() >= getAttackSpeed()) {
        setSpeed(getAttackSpeed());
        setState(MOVING);
      }
    }
    setPosition(getPosition() +
                Position{cosf(getDirection()), sinf(getDirection())} *
                    (getSpeed() * getConfig()->getSimTimeStep()));
    break;

  case DECELERATING:
    setSpeed(getSpeed() - getAcceleration() * getConfig()->getSimTimeStep());
    if (getSpeed() <= 0.0f) {
      setSpeed(0.0f);
      setState(TURNING);
      turnAngleLeft = fabsf(angleDiff);
    } else {
      setPosition(getPosition() +
                  Position{cosf(getDirection()), sinf(getDirection())} *
                      (getSpeed() * getConfig()->getSimTimeStep()));
    }
    break;

  case TURNING:
    if (fabsf(angleDiff) <= angStep) {
      setDirection(desiredDir);
      setState(ACCELERATING);
      turnAngleLeft = 0.0f;
    } else {
      setDirection(getDirection() + (angleDiff > 0 ? angStep : -angStep));
      turnAngleLeft = fabsf(angleDiff) - angStep;
      if (turnAngleLeft < 0.0f)
        turnAngleLeft = 0.0f;
    }
    break;
  }
}

void DroneDetails::setSpeed(float speed) { _speed = speed; };

void DroneDetails::setDirection(float direction) {
  _direction = normalizeAngle(direction);
};

void DroneDetails::setState(DroneState state) { _state = state; };

void DroneDetails::setPosition(Position position) { _position = position; };

void DroneDetails::validateParameters() const {
  if (getAttackSpeed() <= 0.0f || getAccelPath() <= 0.0f ||
      getAltitude() <= 0.0f) {
    throw std::runtime_error("ERROR: Invalid parameters\n");
  }
}
