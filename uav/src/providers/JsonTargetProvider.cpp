#include "JsonTargetProvider.hpp"

#include "Helper.hpp"

JSONTargetProvider::JSONTargetProvider(TimeManagement *timeManagement,
                     const std::string &jsonFilePath)
      : _timeManagement(timeManagement) {
    initJsonTargets(jsonFilePath);
}

void JSONTargetProvider::initJsonTargets(const std::string jsonFilePath) {
  json rawData = loadJsonFile(jsonFilePath);

  _targets.clear();

  int targetCount = rawData["targetCount"];
  for (int i = 0; i < targetCount; ++i) {
    std::vector<Position> targetPositions;
    for (const auto &pos : rawData["targets"][i]["positions"]) {
      targetPositions.emplace_back(Position{pos["x"], pos["y"]});
    }

    _targets.push_back(targetPositions);
  }
}

Position JSONTargetProvider::getTarget(int index) {
  const int sampleCount = static_cast<int>(_targets[index].size());
  int rawIdx = (int)floorf(_timeManagement->getCurrentTime() /
                           _timeManagement->getArrayTimeStep());
  int idx = ((rawIdx % sampleCount) + sampleCount) % sampleCount;
  int next = (idx + 1) % sampleCount;
  float frac = (_timeManagement->getCurrentTime() -
                rawIdx * _timeManagement->getArrayTimeStep()) /
               _timeManagement->getArrayTimeStep();

  return _targets[index][idx] +
         (_targets[index][next] - _targets[index][idx]) * frac;
}
