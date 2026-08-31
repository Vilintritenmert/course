#pragma once

#include <atomic>
#include <mutex>
#include <vector>

#include "interfaces.hpp"

namespace uav {

class ThreadSafeTargetProvider : public ITargetProvider {
private:
  void advanceStep();

  std::vector<std::vector<Coord>> trajectories_;

  std::vector<Target> currentTargets_;
  mutable std::mutex mutex_;

  float arrayTimeStep_ = 1.F;
  float timeScale_ = 1.F;
  int currentIndex_ = 0;

  std::atomic<bool> ready_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stopRequested_{false};

  public:
  ~ThreadSafeTargetProvider() override = default;

  auto load(const std::string &filePath) -> bool override;
  auto getTargetCount() const -> int override {
    return static_cast<int>(trajectories_.size());
  }
  auto getTarget(int index) const -> Target override;

  void setArrayTimeStep(float dt) override { arrayTimeStep_ = dt; }
  void advance(float /*dt*/) override {}
  void reset() override {}

  void setTimeScale(float scale) { timeScale_ = scale; }

  auto isThreadReady() const -> bool { return ready_.load(); }
  void start() { running_.store(true); }
  void stop() { stopRequested_.store(true); }

  void run();

};

} // namespace uav
