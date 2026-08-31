#pragma once

#include <memory>

#include "uav_ai/types.hpp"

namespace uav {

// Спільні дані, з якими працюють усі стани дрона. Складається з посилань на
// поля Drone (щоб стани могли їх змінювати без ручного копіювання назад) та
// конфігу, спільного для всієї симуляції.
struct DroneContext {
  Coord &pos;
  float &direction;
  float &speed;
  float &targetDir;      // куди повертаємось (TURNING / DECELERATING -> TURNING)
  float &turnRemaining;
  float &turnSign;

  float desiredDir = 0.F; // бажаний курс на ціль, обраний солвером цього кроку
  float acceleration = 0.F;
  const DroneConfig &cfg;
};

// Базовий клас стану дрона. Кожен конкретний стан з ДЗ (STOPPED,
// ACCELERATING, DECELERATING, TURNING, MOVING) - окрема реалізація.
class IDroneState {
public:
  virtual ~IDroneState() = default;

  // Виконати логіку стану, повернути наступний стан.
  // Якщо стан не змінився - повернути nullptr (виклик залишить поточний).
  virtual auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> = 0;

  // Скільки часу знадобиться, щоб повністю зупинитись з поточного стану
  // (використовується під час вибору цілі - зміна цілі "коштує" цей час).
  virtual auto timeToStop(const DroneContext &ctx) const -> float {
    (void)ctx;
    return 0.F;
  }

  virtual auto name() const -> const char * = 0;
};

class StateStopped : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto name() const -> const char * override { return "Stopped"; }
};

class StateAccelerating : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Accelerating"; }
};

class StateDecelerating : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Decelerating"; }
};

class StateTurning : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Turning"; }
};

class StateMoving : public IDroneState {
public:
  auto execute(DroneContext &ctx) -> std::unique_ptr<IDroneState> override;
  auto timeToStop(const DroneContext &ctx) const -> float override;
  auto name() const -> const char * override { return "Moving"; }
};

} // namespace uav
