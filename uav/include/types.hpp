#pragma once
#include <cfloat>
#include <cmath>
#include <string>
#include <algorithm>

class TimeManagement;
class ITargetProvider;

inline float normalizeAngle(float a) {
  while (a > (float)M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}

enum DroneState {
  STOPPED = 0,
  ACCELERATING = 1,
  DECELERATING = 2,
  TURNING = 3,
  MOVING = 4
};

class Position {
private:
  float _x, _y;

public:
  Position(const float x = 0, const float y = 0) : _x(x), _y(y){};

  float getX() const { return _x; };
  float getY() const { return _y; };

  void setX(const float x) { _x = x; };
  void setY(const float y) { _y = y; };

  Position operator+(const Position &o) const {
    return Position(_x + o._x, _y + o._y);
  };

  Position operator-(const Position &o) const {
    return Position(_x - o._x, _y - o._y);
  };

  Position operator*(const float s) { return Position(_x * s, _y * s); };
  Position operator/(const float s) { return Position(_x / s, _y / s); }
  bool operator==(const Position &o) { return _x == o._x && _y == o._y; }

  static float distance(const Position position) {
    return sqrtf(position._x * position._x + position._y * position._y);
  }

  static Position normalize(Position c) {
    float l = Position::distance(c);
    return l > 1e-9f ? c / l : Position(0.0f, 0.0f);
  }

  ~Position() = default;
};

class AmmoParams {
private:
  std::string  _name;
  float _mass = 0, _drag = 0, _lift = 0;

public:
  AmmoParams(std::string name, float mass, float drag, float lift)
      : _name(name), _mass(mass), _drag(drag), _lift(lift) {}

  std::string getName() const { return _name; }  ;

  float getMass() const { return _mass; };
  float getDrag() const { return _drag; };
  float getLift() const { return _lift; };

  ~AmmoParams() = default;
};

class DroneDetails {
private:
  Position _position;
  AmmoParams _ammo;
  float _speed = 0.0f;
  DroneState _state = STOPPED;
  float _altitude, _direction, _attackSpeed, _angularSpeed, _accelPath;

public:
  DroneDetails(Position position, AmmoParams ammo, float altitude,
              float direction, float attackSpeed, float angularSpeed,
              float accelPath)
      : _position(position), _ammo(ammo), _altitude(altitude),
        _direction(direction), _attackSpeed(attackSpeed),
        _angularSpeed(angularSpeed), _accelPath(accelPath) {}

  Position getPosition() const { return _position; };

  AmmoParams getAmmo() const { return _ammo; };

  float getSpeed() const { return _speed; };

  float getAltitude() const { return _altitude; };

  float getDirection() const { return _direction; };

  float getAttackSpeed() const { return _attackSpeed; };

  float getAccelPath() const { return _accelPath; };

  float getAcceleration() const {
    return _attackSpeed * _attackSpeed / (2.0f * _accelPath);
  };

  float getAngularSpeed() const { return _angularSpeed; };

  DroneState getState() const { return _state; };

  AmmoParams getAmmoParams() const { return _ammo; };

  int selectTarget(ITargetProvider *targetProvider, TimeManagement *timeManager,
                   float zd, int targetCount, int currentTargetIdx,
                   float turnTimeLeft, Position outDrop[], Position outPred[]);

  void updateDrone(Position &pos, float &dir, float &speed, DroneState &state,
                   float desiredDir, float dt, float attackSpeed, float accel,
                   float angularSpeed, float turnThreshold,
                   float &turnAngleLeft);

  ~DroneDetails() = default;
};

class TimeManagement {
  private:
    float _simTimeStep, _arrayTimeStep;
    float _currentTime = 0.0f;

  public:
    TimeManagement(float simTimeStep, float arrayTimeStep)
        : _simTimeStep(simTimeStep), _arrayTimeStep(arrayTimeStep) {} 

    float getCurrentTime() { return _currentTime; };

    float getArrayTimeStep() { return _arrayTimeStep; };
    
    int getTimeStep() { return _simTimeStep / _arrayTimeStep; };

    void tick() { _currentTime += _simTimeStep; };
};

struct SimStep {
  Position pos;
  float direction;
  int state, targetIdx;
  Position dropPoint, aimPoint, predictedTarget;
};