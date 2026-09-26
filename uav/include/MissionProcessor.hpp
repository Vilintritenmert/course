#pragma once

#include <cmath>
#include <memory>
#include <nlohmann/json.hpp>

#include "ComponentFactory.hpp"
#include "DroneContext.hpp"
#include "Helper.hpp"
#include "IBallisticSolver.hpp"
#include "IConfigLoader.hpp"
#include "IMissionState.hpp"
#include "ITargetProvider.hpp"
#include "MissionEngagement.hpp"
#include "SimStep.hpp"
#include "TimeManagement.hpp"

using json = nlohmann::json;

class MissionProcessor {
  friend class StateNavigating;
  friend class StateEngaging;

private:
  static constexpr int MAX_STEPS = 10000;

  std::shared_ptr<ConfigLoaderOptions> _configLoaderOptions;
  std::unique_ptr<ComponentFactory> _factory;
  std::shared_ptr<DroneContext> _droneDetails;
  std::shared_ptr<TimeManagement> _timeManagement = nullptr;
  std::unique_ptr<ITargetProvider> _targetProvider;
  std::unique_ptr<IBallisticSolver> _ballisticSolver;
  std::shared_ptr<Config> _config;

  std::vector<SimStep> _steps;
  int _totalSteps = 0;
  int _currentTarget = -1;
  int _step = 0;

  float _turnAngleLeft = 0.0f;

  std::unique_ptr<IMissionState> _state;

  void fillOutputJson(json &output) const;

  void init();

public:
  MissionProcessor(std::shared_ptr<ConfigLoaderOptions> configLoaderOptions);

  ~MissionProcessor() = default;

  bool hasNext() const;

  void changeSolver(SolverType solverType);

  void reset();

  void storeSimulation();

  void step();
};