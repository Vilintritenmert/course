#include "thread_safe_target_provider.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace uav {

auto ThreadSafeTargetProvider::load(const std::string &filePath) -> bool {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    std::cerr << "Error: cannot open `" << filePath << "`\n";
    return false;
  }

  json j;
  file >> j;

  const int targetCount = j["targetCount"];

  trajectories_.clear();
  trajectories_.reserve(targetCount);
  for (const auto &target : j["targets"]) {
    std::vector<Coord> trajectory;
    trajectory.reserve(target["positions"].size());
    for (const auto &position : target["positions"]) {
      trajectory.push_back(Coord{position["x"], position["y"]});
    }
    trajectories_.push_back(std::move(trajectory));
  }

  return true;
}

auto ThreadSafeTargetProvider::getTarget(int index) const -> Target {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index < 0 || index >= static_cast<int>(currentTargets_.size())) {
    return {};
  }
  return currentTargets_[index];
}

void ThreadSafeTargetProvider::advanceStep() {
  const int nextIndex = currentIndex_ + 1;

  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < trajectories_.size(); ++i) {
    const auto &traj = trajectories_[i];
    if (traj.empty()) {
      continue;
    }
    const int n = static_cast<int>(traj.size());
    const int idx = ((currentIndex_ % n) + n) % n;
    const int nidx = ((nextIndex % n) + n) % n;
    currentTargets_[i].pos = traj[nidx];
    currentTargets_[i].velocity = (traj[nidx] - traj[idx]) / arrayTimeStep_;
  }
  currentIndex_ = nextIndex;
}

void ThreadSafeTargetProvider::run() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    currentTargets_.assign(trajectories_.size(), Target{});
    for (std::size_t i = 0; i < trajectories_.size(); ++i) {
      if (!trajectories_[i].empty()) {
        currentTargets_[i].pos = trajectories_[i][0];
      }
    }
  }
  currentIndex_ = 0;

  ready_.store(true);
  while (!running_.load() && !stopRequested_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!stopRequested_.load()) {
    std::this_thread::sleep_for(
        std::chrono::duration<float>(arrayTimeStep_ / timeScale_));
    if (stopRequested_.load()) {
      break;
    }
    advanceStep();
  }
}

} // namespace uav
