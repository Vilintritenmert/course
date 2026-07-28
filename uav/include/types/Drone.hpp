#pragma once

#include "AmmoParams.hpp"
#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"
#include "Position.hpp"

enum DroneState {
  STOPPED = 0,
  ACCELERATING = 1,
  DECELERATING = 2,
  TURNING = 3,
  MOVING = 4
};

class DroneDetails {
private:
  const Config *_config;
  const AmmoConfig *_ammoConfig;

  Position _position;
  AmmoParams _ammo;
  float _speed = 0.0f;
  DroneState _state = STOPPED;
  float _altitude, _direction, _attackSpeed, _angularSpeed, _accelPath;

public:
  DroneDetails(const Config *config, const AmmoConfig *ammoConfigObj);

  float normalizeAngle(float a);

  Position getPosition() const { return _position; };

  AmmoParams getAmmo() const { return _ammo; };

  const Config *getConfig() const { return _config; };

  float getSpeed() const { return _speed; };

  float getAltitude() const { return _altitude; };

  float getDirection() const { return _direction; };

  float getAttackSpeed() const { return _attackSpeed; };

  float getAccelPath() const { return _accelPath; };

  float getAcceleration() const;

  float getAngularSpeed() const { return _angularSpeed; };

  float getSimTimeStep() const;

  float getTurnThreshold() const;

  DroneState getState() const { return _state; };

  AmmoParams getAmmoParams() const { return _ammo; };

  void setSpeed(float speed);

  void setDirection(float direction);

  void setState(DroneState state);

  void setPosition(Position position);

  int selectTarget(ITargetProvider *targetProvider, int currentTargetIdx,
                   float turnTimeLeft);

  void updateDrone(float desiredDir, float &turnAngleLeft);

  void reset();

  void validateParameters() const;

  ~DroneDetails() = default;
};
