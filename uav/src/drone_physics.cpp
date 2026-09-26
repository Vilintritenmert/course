#include "drone_physics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace uav {

void DronePhysics::init(const DroneConfig &cfg) {
  cfg_ = cfg;
  std::lock_guard<std::mutex> lock(mutex_);
  pos_ = cfg_.startPos;
  direction_ = cfg_.initialDir;
  speed_ = 0.F;
  mode_ = DroneMode::Stopped;
  elapsedTime_ = 0.F;
  currentCommand_ = {DroneMode::Stopped, 0.F};
}

void DronePhysics::stepOnce(float dt) {
  const float acceleration =
      (cfg_.attackSpeed * cfg_.attackSpeed) / (2.F * cfg_.accelPath);

  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = currentCommand_.mode;
  const Coord dir = {std::cos(direction_), std::sin(direction_)};

  switch (mode_) {
  case DroneMode::Stopped:
    speed_ = 0.F;
    break;
  case DroneMode::Accelerating:
    speed_ = std::min(cfg_.attackSpeed, speed_ + acceleration * dt);
    pos_ = pos_ + dir * (speed_ * dt);
    break;
  case DroneMode::Decelerating:
    speed_ = std::max(0.F, speed_ - acceleration * dt);
    pos_ = pos_ + dir * (speed_ * dt);
    break;
  case DroneMode::Turning:
    direction_ = normalizeAngle(direction_ + currentCommand_.angleSpeed * dt);
    break;
  case DroneMode::Moving:
    speed_ = cfg_.attackSpeed;
    pos_ = pos_ + dir * (speed_ * dt);
    break;
  }

  elapsedTime_ += dt;
}

void DronePhysics::run() {
  ready_.store(true);
  while (!running_.load() && !stopRequested_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const float dt = cfg_.physicsTimeStep;
  while (!stopRequested_.load()) {
    if (auto cmd = commands_.tryPop()) {
      currentCommand_ = *cmd;
    }
    stepOnce(dt);
    std::this_thread::sleep_for(
        std::chrono::duration<float>(dt / cfg_.timeScale));
  }
}

auto DronePhysics::getTelemetry() const -> DroneTelemetry {
  std::lock_guard<std::mutex> lock(mutex_);
  DroneTelemetry t;
  t.pos = pos_;
  t.direction = direction_;
  t.speed = {std::cos(direction_) * speed_, std::sin(direction_) * speed_};
  t.mode = mode_;
  t.timeSecSinceStart = elapsedTime_;
  return t;
}

} // namespace uav
