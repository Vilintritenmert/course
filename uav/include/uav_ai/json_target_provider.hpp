#pragma once

#include <vector>

#include "uav_ai/interfaces.hpp"

namespace uav {

// Завантажує траєкторії цілей з targets.json (масив позицій з кроком
// arrayTimeStep для кожної цілі) і на кожен getTarget(idx) інтерполює
// позицію/швидкість цілі на поточний внутрішній час provider'а.
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

  // Крок часу, який використовується для чисельної оцінки швидкості цілі
  // (скінченна різниця interpolate(t+dt) - interpolate(t)). За замовчуванням
  // рівний arrayTimeStep, але зазвичай виставляється рівним simTimeStep
  // симуляції для точнішої оцінки.
  void setVelocityTimeStep(float dt) { velocityDt_ = dt; }

private:
  auto interpolate(int targetIdx, float t) const -> Coord;

  std::vector<std::vector<Coord>> trajectories_;
  float arrayTimeStep_ = 1.F;
  float velocityDt_ = 0.1F;
  float currentTime_ = 0.F;
};

} // namespace uav
