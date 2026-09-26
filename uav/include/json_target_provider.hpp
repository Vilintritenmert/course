#pragma once

#include <vector>

#include "interfaces.hpp"

namespace uav {

class JsonTargetProvider : public ITargetProvider {
public:
  ~JsonTargetProvider() override = default;

  auto load(const std::string &filePath) -> bool override;
  auto getTargetCount() const -> int override {
    return static_cast<int>(trajectories_.size());
  }
  auto getTarget(int index) const -> Target override;

  void advance(float dt) override { currentTime_ += dt; }
  void reset() override { currentTime_ = 0.F; }
  void setArrayTimeStep(float dt) override { arrayTimeStep_ = dt; }

  void setVelocityTimeStep(float dt) { velocityDt_ = dt; }

private:
  auto interpolate(int targetIdx, float t) const -> Coord;

  std::vector<std::vector<Coord>> trajectories_;
  float arrayTimeStep_ = 1.F;
  float velocityDt_ = 0.1F;
  float currentTime_ = 0.F;
};

} // namespace uav
