#pragma once

#include <atomic>
#include <mutex>

#include "thread_safe_queue.hpp"
#include "types.hpp"

namespace uav {

class DronePhysics {
private:
  void stepOnce(float dt);

  DroneConfig cfg_;

  Coord pos_;
  float direction_ = 0.F;
  float speed_ = 0.F;
  DroneMode mode_ = DroneMode::Stopped;
  float elapsedTime_ = 0.F;
  mutable std::mutex mutex_;

  DroneCommand currentCommand_{DroneMode::Stopped, 0.F};
  ThreadSafeQueue<DroneCommand> commands_;

  std::atomic<bool> ready_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stopRequested_{false};

public:
  void init(const DroneConfig &cfg);

  auto isThreadReady() const -> bool { return ready_.load(); }

  void start() { running_.store(true); }

  void stop() { stopRequested_.store(true); }

  void run();

  void sendCommand(const DroneCommand &command) { commands_.push(command); }

  auto getTelemetry() const -> DroneTelemetry;
};

} // namespace uav
