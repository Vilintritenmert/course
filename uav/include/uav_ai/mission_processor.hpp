#pragma once

#include <memory>
#include <string>
#include <vector>

#include "uav_ai/interfaces.hpp"

namespace uav {

class IDroneState;

// Основний алгоритм симуляції: володіє компонентами через unique_ptr
// (передаються за std::move) - викликаючий код більше не відповідає за
// їхнє знищення. changeSolver() дозволяє підмінити солвер на льоту, не
// змінюючи решту логіки.
class MissionProcessor {
public:
  MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                    std::unique_ptr<ITargetProvider> provider,
                    std::unique_ptr<IBallisticSolver> solver);
  ~MissionProcessor();

  // Завантажує конфіг/боєприпас через configLoader, цілі через provider
  // (targetsPath), ініціалізує солвер і стан дрона. dataDir - директорія з
  // config.json/ammo.json.
  auto init(const std::string &dataDir, const std::string &targetsPath) -> bool;

  auto hasNext() const -> bool;
  void step();
  void reset();

  // Підміна солвера "на льоту" (Стратегія). Володіння переходить до
  // MissionProcessor, старий солвер звільняється автоматично.
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  auto getHistory() const -> const std::vector<SimStep> & { return history_; }
  auto getStepCount() const -> int { return stepCount_; }

  void writeOutput(const std::string &outputPath) const;

private:
  void resetDrone();

  std::unique_ptr<IConfigLoader> configLoader_;
  std::unique_ptr<ITargetProvider> provider_;
  std::unique_ptr<IBallisticSolver> solver_;

  DroneConfig config_;
  AmmoParams ammo_;

  Drone drone_;
  std::unique_ptr<IDroneState> state_;
  float currentTime_ = 0.F;
  int selectedTarget_ = -1;
  int stepCount_ = 0;
  bool finished_ = false;
  bool initialized_ = false;

  std::vector<SimStep> history_;
};

} // namespace uav
