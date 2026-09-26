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

  float physicsTimeStep = 0.01F;
  float timeScale = 10.F;
};

enum class DroneMode { Stopped, Accelerating, Decelerating, Turning, Moving };

inline auto droneModeName(DroneMode mode) -> const char * {
  switch (mode) {
  case DroneMode::Stopped:
    return "Stopped";
  case DroneMode::Accelerating:
    return "Accelerating";
  case DroneMode::Decelerating:
    return "Decelerating";
  case DroneMode::Turning:
    return "Turning";
  case DroneMode::Moving:
    return "Moving";
  }
  return "Unknown";
}

struct DroneCommand {
  DroneMode mode = DroneMode::Stopped;
  float angleSpeed = 0.F;
};

struct DroneTelemetry {
  Coord pos;
  Coord speed; // вектор швидкості (величина * напрямок)
  float direction = 0.F;
  DroneMode mode = DroneMode::Stopped;
  float timeSecSinceStart = 0.F;
};

struct SimStep {
  Coord pos;
  float direction = 0.F;
  std::string state; // ім'я поточного DroneMode з телеметрії
  int targetIdx = -1;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
  float timeSecSinceStart = 0.F;
};

struct Target {
  Coord pos;
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
