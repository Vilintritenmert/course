#pragma once

#include <cmath>
#include <vector>

#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "factory.hpp"
#include "json_helper.hpp"
#include "target_provider.hpp"
#include "types.hpp"

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

class MissionPlanner {
private:
  static constexpr int MAX_STEPS = 10000;

  ConfigLoaderOptions _configLoaderOptions;
  Factory *_factory;
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

  void fillOutputJson(json &output) const {
    output["totalSteps"] = _totalSteps;
    output["steps"] = json::array();

    for (int i = 0; i < _totalSteps; ++i) {
      json stepJson;
      stepJson["position"] = {{"x", _steps[i].pos.getX()},
                              {"y", _steps[i].pos.getY()}};
      stepJson["direction"] = _steps[i].direction;
      stepJson["state"] = _steps[i].state;
      stepJson["targetIndex"] = _steps[i].targetIdx;
      stepJson["dropPoint"] = {{"x", _steps[i].dropPoint.getX()},
                               {"y", _steps[i].dropPoint.getY()}};
      stepJson["aimPoint"] = {{"x", _steps[i].aimPoint.getX()},
                              {"y", _steps[i].aimPoint.getY()}};
      stepJson["predictedTarget"] = {{"x", _steps[i].predictedTarget.getX()},
                                     {"y", _steps[i].predictedTarget.getY()}};
      output["steps"].push_back(stepJson);
    }
  }

  void init() {
    _factory = new Factory(_configLoaderOptions);

    IConfigLoader *configLoader = _factory->getConfigLoader();
    _config = configLoader->getConfig();
    const AmmoConfig *ammoConfig = configLoader->getAmmoConfig();

    _droneDetails = new DroneDetails(_config, ammoConfig);

    _timeManagement = _factory->getTimeManagement();
    _targetProvider = _factory->createProvider(ProviderType::JSON);
    _ballisticSolver = _factory->createSolver(SolverType::ANALYTICAL);
  }

public:
  MissionPlanner(ConfigLoaderOptions configLoaderOptions)
      : _configLoaderOptions(configLoaderOptions) {
    init();
  }

  ~MissionPlanner() {
    delete _droneDetails;
    delete _factory;
  }

  bool hasNext() const { return _step < MAX_STEPS; }

  void changeSolver(SolverType solverType) {
    _ballisticSolver = _factory->createSolver(solverType);
  }

  void reset() {
    _step = 0;
    _totalSteps = 0;
    _currentTarget = -1;

    _steps.clear();

    _droneDetails->reset();
    _timeManagement->reset();
    _stagingMode = false;
  }

  void storeSimulation() {
    json output;
    fillOutputJson(output);
    storeJson(_configLoaderOptions.getResultPath(), output);
  }

  void step() {
    _turnAngleLeft = (_droneDetails->getState() == TURNING)
                         ? (_turnAngleLeft / _droneDetails->getAngularSpeed())
                         : 0.0f;

    int best = _droneDetails->selectTarget(_targetProvider, _currentTarget,
                                           _turnAngleLeft);
    if (best == -1) {
      return;
    }

    _currentTarget = best;
    Position predPos = _targetProvider->getTarget(_currentTarget);
    float horizonDistance = _ballisticSolver->computeHorizDist(
        _ballisticSolver->computeFlightTime(_droneDetails), _droneDetails);

    _steps.resize(_step + 1);
    _steps[_step].pos = _droneDetails->getPosition();
    _steps[_step].direction = _droneDetails->getDirection();
    _steps[_step].state = static_cast<int>(_droneDetails->getState());
    _steps[_step].targetIdx = _currentTarget;
    _steps[_step].dropPoint = _targetProvider->getTarget(_currentTarget);
    _steps[_step].predictedTarget = predPos;

    _steps[_step].aimPoint = _droneDetails->getPosition() +
                             Position{cosf(_droneDetails->getDirection()),
                                      sinf(_droneDetails->getDirection())} *
                                 horizonDistance;

    DEBUG("Step " << step << " pos=(" << _droneDetails->getPosition().getX()
                  << "," << _droneDetails->getPosition().getY()
                  << ") target=" << _currentTarget
                  << " state=" << static_cast<int>(_droneDetails->getState()));

    float distToPred =
        Position::distance(predPos - _droneDetails->getPosition());

    if (distToPred < horizonDistance) {
      _stagingMode = true;
    } else if (distToPred >=
               horizonDistance + _droneDetails->getConfig()->getAccelPath()) {
      _stagingMode = false;
    }

    if (!_stagingMode && _droneDetails->getState() == MOVING &&
        distToPred <= horizonDistance + _config->getHitRadius()) {
      ++_step;
      return;
    }

    Position navPos;
    if (_stagingMode && distToPred > 1e-3f) {
      navPos = predPos +
               Position::normalize(_droneDetails->getPosition() - predPos) *
                   (horizonDistance + _config->getAccelPath());
    } else {
      navPos = predPos;
    }

    float desiredDir =
        atan2f(navPos.getY() - _droneDetails->getPosition().getY(),
               navPos.getX() - _droneDetails->getPosition().getX());

    _droneDetails->updateDrone(desiredDir, _turnAngleLeft);

    _timeManagement->tick();
    ++_step;
    _totalSteps = _step;
  }
};