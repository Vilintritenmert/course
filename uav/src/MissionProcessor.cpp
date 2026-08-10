#include <memory>

#include "MissionProcessor.hpp"
#include "StateNavigating.hpp"

MissionProcessor::MissionProcessor(
    std::shared_ptr<ConfigLoaderOptions> configLoaderOptions)
    : _configLoaderOptions(configLoaderOptions) {
  init();
}

void MissionProcessor::fillOutputJson(json &output) const {
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

void MissionProcessor::init() {
  _factory = std::make_unique<ComponentFactory>(_configLoaderOptions);

  IConfigLoader *configLoader = _factory->getConfigLoader().get();
  _config = configLoader->getConfig();
  std::shared_ptr<AmmoConfig> ammoConfig = configLoader->getAmmoConfig();

  _droneDetails = std::make_shared<DroneContext>(_config, ammoConfig);

  _timeManagement = _factory->getTimeManagement();
  _targetProvider = _factory->createProvider(ProviderType::JSON);
  _ballisticSolver = _factory->createSolver(SolverType::ANALYTICAL);

  _state = std::make_unique<StateNavigating>();
}

bool MissionProcessor::hasNext() const { return _step < MAX_STEPS; }

void MissionProcessor::changeSolver(SolverType solverType) {
  _ballisticSolver = _factory->createSolver(solverType);
}

void MissionProcessor::reset() {
  _step = 0;
  _totalSteps = 0;
  _currentTarget = -1;

  _steps.clear();

  _droneDetails->reset();
  _timeManagement->reset();

  _state = std::make_unique<StateNavigating>();
}

void MissionProcessor::storeSimulation() {
  json output;
  fillOutputJson(output);
  storeJson(_configLoaderOptions->getResultPath(), output);
}

void MissionProcessor::step() {
  _turnAngleLeft =
      (_droneDetails->getState() == TURNING)
          ? (_turnAngleLeft / _droneDetails->getConfig()->getAngularSpeed())
          : 0.0f;

  int best = _droneDetails->selectTarget(_targetProvider.get(), _currentTarget,
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

  float distToPrediction =
      Position::distance(predPos - _droneDetails->getPosition());

  MissionEngagement engagement{predPos, horizonDistance, distToPrediction,
                               distToPrediction < horizonDistance};

  auto next = _state->execute(*this, engagement);
  if (next) {
    _state = std::move(next);
  }
}
