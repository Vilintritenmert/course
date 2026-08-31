#include "json_target_provider.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace uav {

auto JsonTargetProvider::load(const std::string &filePath) -> bool {
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

auto JsonTargetProvider::interpolate(int targetIdx, float t) const -> Coord {
  const int timeSteps = static_cast<int>(trajectories_[targetIdx].size());
  const auto index = static_cast<int>(std::floor(t / arrayTimeStep_));
  const int idx = ((index % timeSteps) + timeSteps) % timeSteps;
  const int next = (idx + 1) % timeSteps;
  const float frac =
      (t - static_cast<float>(index) * arrayTimeStep_) / arrayTimeStep_;
  return trajectories_[targetIdx][idx] +
         (trajectories_[targetIdx][next] - trajectories_[targetIdx][idx]) * frac;
}

auto JsonTargetProvider::getTarget(int index) const -> Target {
  Target target;
  target.pos = interpolate(index, currentTime_);
  const Coord next = interpolate(index, currentTime_ + velocityDt_);
  target.velocity = (next - target.pos) / velocityDt_;
  return target;
}

} // namespace uav
