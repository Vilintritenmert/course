#pragma once

#include "AmmoParams.hpp"
#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"
#include "Position.hpp"
#include <memory>

enum DroneState {
  STOPPED = 0,
  ACCELERATING = 1,
  DECELERATING = 2,
  TURNING = 3,
  MOVING = 4
};

class IDroneState;

class DroneContext {
private:
  const std::shared_ptr<Config> _config;
  const std::shared_ptr<AmmoConfig> _ammoConfig;

  Position _position;
  AmmoParams _ammo;

  float _speed = 0.0f;
  float _disiredDirection = 0.0f;
  float _targetDirection = 0.0f;
  float _turnRemaining = 0.0f;

  std::unique_ptr<IDroneState> _currentState;
  float _direction, _accelPath;

public:
  DroneContext(const std::shared_ptr<Config> config,
               const std::shared_ptr<AmmoConfig> ammoConfigObj);

  float normalizeAngle(float a) const;

  Position getPosition() const { return _position; };

  AmmoParams getAmmo() const { return _ammo; };

  std::shared_ptr<Config> getConfig() const { return _config; };

  float getSpeed() const { return _speed; };

  float getDirection() const { return _direction; };

  float getAccelPath() const { return _accelPath; };

  float getDisiredDirection() const { return _disiredDirection; };

  float getAcceleration() const;

   float getAngleDelta() const;

  float getTargetDirection() const {
    return _targetDirection;
  };

  DroneState getState() const;

  float getTurnRemaining() const { return _turnRemaining; };

  AmmoParams getAmmoParams() const { return _ammo; };

  void setSpeed(float speed);

  void setDirection(float direction);

  void setDisiredDirection(float disiredDirection) {
    _disiredDirection = disiredDirection;
  };

  void setPosition(Position position);

  void setTurnRemaining(float turnRemaining);

  void setTargetDirection(float targetDirection);

  int selectTarget(ITargetProvider* targetProvider, int currentTargetIdx,
                   float turnTimeLeft);

  void updateDrone(float desiredDir, float &turnAngleLeft);

  void reset();

  void validateParameters() const;

  ~DroneContext();
};
