#pragma once

#include <memory>
#include <string>
#include <vector>

#include "uav_v2/interfaces.hpp"

namespace uav {

class IDroneState;

class MissionProcessor {
public:
  MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                    std::unique_ptr<ITargetProvider> provider,
                    std::unique_ptr<IBallisticSolver> solver);
  ~MissionProcessor();

  auto init(const std::string &dataDir, const std::string &targetsPath) -> bool;

  auto hasNext() const -> bool;
  void step();
  void reset();

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

}
