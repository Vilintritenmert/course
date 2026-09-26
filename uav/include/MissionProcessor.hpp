#pragma once

#include <cmath>
#include <nlohmann/json.hpp>

#include "ComponentFactory.hpp"
#include "Drone.hpp"
#include "Helper.hpp"
#include "IBallisticSolver.hpp"
#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"
#include "SimStep.hpp"
#include "TimeManagement.hpp"

using json = nlohmann::json;

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

class MissionProcessor {
private:
  static constexpr int MAX_STEPS = 10000;

  ConfigLoaderOptions _configLoaderOptions;
  ComponentFactory *_factory;
  DroneDetails *_droneDetails;
  TimeManagement *_timeManagement;
  ITargetProvider *_targetProvider;
  IBallisticSolver *_ballisticSolver;
  const Config *_config;

  std::vector<SimStep> _steps;
  int _totalSteps = 0;
  int _currentTarget = -1;
  int _step = 0;

  bool _stagingMode = false;
  float _turnAngleLeft = 0.0f;

  void fillOutputJson(json &output) const;

  void init();

public:
  MissionProcessor(ConfigLoaderOptions configLoaderOptions);

  ~MissionProcessor();

  bool hasNext() const;

  void changeSolver(SolverType solverType);

  void reset();

  void storeSimulation();

  void step();
};