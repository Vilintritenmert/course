#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "interfaces.hpp"

namespace uav {

class IDroneState;
class DronePhysics;

class MissionProcessor {
private:
  DroneConfig config_;
  AmmoParams ammo_;

  std::unique_ptr<IConfigLoader> configLoader_;
  ITargetProvider *provider_;
  DronePhysics *physics_;
  std::unique_ptr<IBallisticSolver> solver_;

  std::unique_ptr<IDroneState> state_;
  float currentTime_ = 0.F;
  int selectedTarget_ = -1;
  int stepCount_ = 0;
  bool finished_ = false;
  bool initialized_ = false;

  std::vector<SimStep> history_;

  std::atomic<bool> ready_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> stopRequested_{false};

public:
  MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                   ITargetProvider *provider, DronePhysics *physics,
                   std::unique_ptr<IBallisticSolver> solver);
  ~MissionProcessor();

  auto init(const std::string &dataDir) -> bool;

  auto hasNext() const -> bool;
  void step();
  void reset();

  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  auto getHistory() const -> const std::vector<SimStep> & { return history_; }
  auto getStepCount() const -> int { return stepCount_; }

  void writeOutput(const std::string &outputPath) const;

  auto isThreadReady() const -> bool { return ready_.load(); }

  void start() { running_.store(true); }

  void stop() { stopRequested_.store(true); }

  void run();
};

} // namespace uav
