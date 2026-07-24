#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "ballistic_solver.hpp"
#include "config_loader.hpp"
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

  DroneDetails _droneDetails;
  TimeManagement *_timeManagement;
  ITargetProvider *_targetProvider;
  IBallisticSolver *_ballisticSolver;
  const Config *_config;
  std::string _dataFolderPath;
  std::vector<SimStep> _steps;
  int _totalSteps = 0;
  int _currentTarget = -1;

  void validateParameters() const {
    if (_droneDetails.getAttackSpeed() <= 0.0f ||
        _droneDetails.getAccelPath() <= 0.0f ||
        _droneDetails.getAltitude() <= 0.0f) {
      throw std::runtime_error("ERROR: Invalid parameters\n");
    }
  }

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

public:
  MissionPlanner(DroneDetails droneDetails, IBallisticSolver *ballisticSolver,
                 ITargetProvider *targetProvider, TimeManagement *timeManagement,
                 const Config *config, const std::string &dataFolderPath)
      : _droneDetails(droneDetails), _timeManagement(timeManagement),
        _targetProvider(targetProvider), _ballisticSolver(ballisticSolver),
        _config(config), _dataFolderPath(dataFolderPath) {}

  int runSimulation() {
    validateParameters();

    float accel = _droneDetails.getAttackSpeed() * _droneDetails.getAttackSpeed() /
                  (2.0f * _droneDetails.getAccelPath());
    float horizonDistance = _ballisticSolver->computeHorizDist(
        _ballisticSolver->computeFlightTime(_droneDetails), _droneDetails);

    int targetCount = _targetProvider->getTargetCount();
    std::vector<Position> allDrop(targetCount);
    std::vector<Position> allPred(targetCount);

    Position dronePos = _droneDetails.getPosition();
    float droneDir = _droneDetails.getDirection();
    float droneSpeed = 0.0f;
    DroneState droneState = STOPPED;
    float turnAngleLeft = 0.0f;
    int step = 0;
    bool stagingMode = false;

    while (step < MAX_STEPS) {
      float turnTimeLeft = (droneState == TURNING)
                               ? (turnAngleLeft / _droneDetails.getAngularSpeed())
                               : 0.0f;

      int best = _droneDetails.selectTarget(
          _targetProvider, _timeManagement, _config->getAltitude(), targetCount,
          _currentTarget, turnTimeLeft, allDrop.data(), allPred.data());
      if (best == -1) {
        break;
      }

      _currentTarget = best;
      Position predPos = allPred[_currentTarget];

      _steps.resize(step + 1);
      _steps[step].pos = dronePos;
      _steps[step].direction = droneDir;
      _steps[step].state = static_cast<int>(droneState);
      _steps[step].targetIdx = _currentTarget;
      _steps[step].dropPoint = allDrop[_currentTarget];
      _steps[step].predictedTarget = predPos;
      _steps[step].aimPoint =
          dronePos + Position{cosf(droneDir), sinf(droneDir)} * horizonDistance;

      DEBUG("Step " << step << " pos=(" << dronePos.getX() << ","
                     << dronePos.getY() << ") target=" << _currentTarget
                     << " state=" << static_cast<int>(droneState));

      float distToPred = Position::distance(predPos - dronePos);

      if (distToPred < horizonDistance) {
        stagingMode = true;
      } else if (distToPred >= horizonDistance + _config->getAccelPath()) {
        stagingMode = false;
      }

      if (!stagingMode && droneState == MOVING &&
          distToPred <= horizonDistance + _config->getHitRadius()) {
        ++step;
        break;
      }

      Position navPos;
      if (stagingMode && distToPred > 1e-3f) {
        navPos = predPos + Position::normalize(dronePos - predPos) *
                               (horizonDistance + _config->getAccelPath());
      } else {
        navPos = predPos;
      }

      float desiredDir = atan2f(navPos.getY() - dronePos.getY(),
                                navPos.getX() - dronePos.getX());
      _droneDetails.updateDrone(dronePos, droneDir, droneSpeed, droneState,
                                desiredDir, _config->getSimTimeStep(),
                                _config->getAttackSpeed(), accel,
                                _config->getAngularSpeed(),
                                _config->getTurnThreshold(), turnAngleLeft);

      _timeManagement->tick();
      ++step;
    }

    _totalSteps = step;
    LOG("Simulation complete. Steps: " << _totalSteps
                                       << " Target: " << _currentTarget);
    storeSimulation(_dataFolderPath + "/simulation.json");
    return 0;
  }

  void storeSimulation(const std::string &filename) {
    json output;
    fillOutputJson(output);
    storeJson(filename, output);
  }
};