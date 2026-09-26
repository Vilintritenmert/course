#pragma once

#include <cmath>
#include <limits>
#include <string>

namespace uav {

constexpr double GRAVITY_ACCELERATION = 9.81;
constexpr double EPSILON = 1e-9;
constexpr int MAX_STEPS = 10000;

struct Coord {
  float x = 0.F;
  float y = 0.F;

  auto operator+(const Coord &other) const -> Coord {
    return {x + other.x, y + other.y};
  }

  auto operator-(const Coord &other) const -> Coord {
    return {x - other.x, y - other.y};
  }

  auto operator*(float s) const -> Coord { return {x * s, y * s}; }

  auto operator/(float s) const -> Coord { return {x / s, y / s}; }

  auto operator==(const Coord &other) const -> bool {
    return x == other.x && y == other.y;
  }
};

inline auto length(Coord c) -> float { return std::hypot(c.x, c.y); }

inline auto normalize(Coord c) -> Coord {
  const float len = length(c);
  if (len < static_cast<float>(EPSILON)) {
    return {0.F, 0.F};
  }
  return c / len;
}

// Приводить кут до діапазону (-pi, pi].
inline auto normalizeAngle(float angle) -> float {
  while (angle > static_cast<float>(M_PI)) {
    angle -= 2.F * static_cast<float>(M_PI);
  }
  while (angle <= -static_cast<float>(M_PI)) {
    angle += 2.F * static_cast<float>(M_PI);
  }
  return angle;
}

struct AmmoParams {
  std::string name;
  float mass = 0.F;
  float drag = 0.F;
  float lift = 0.F;
};

struct DroneConfig {
  Coord startPos;
  float altitude = 0.F;
  float initialDir = 0.F;
  float attackSpeed = 0.F;
  float accelPath = 0.F;
  std::string ammoName;
  float arrayTimeStep = 0.F;
  float simTimeStep = 0.F;
  float hitRadius = 0.F;
  float angularSpeed = 0.F;
  float turnThreshold = 0.F;
};

// Стан дрона під час симуляції (позиція, курс, швидкість). Фаза (STOPPED /
// ACCELERATING / ...) більше не зберігається тут як enum - нею керує
// стейт-машина класів IDroneState (див. drone_state.hpp).
struct Drone {
  Coord pos;
  float dir = 0.F;
  float speed = 0.F;

  // Використовується під час TURNING: куди повертаємось, скільки часу лишилось.
  float turnGoalDir = 0.F;
  float turnRemaining = 0.F;
  float turnSign = 1.F;
};

struct SimStep {
  Coord pos;
  float direction = 0.F;
  std::string state; // ім'я поточного стану (IDroneState::name())
  int targetIdx = -1;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};

// Знімок цілі в поточний момент часу симуляції: позиція та швидкість,
// які повертає ITargetProvider для конкретного targetIdx.
struct Target {
  Coord position;
  Coord velocity;
};

struct TargetCandidate {
  bool valid = false;
  float totalTime = std::numeric_limits<float>::max();
  float heading = 0.F;
  Coord releasePoint;
  Coord predictedTarget;
};

} // namespace uav
