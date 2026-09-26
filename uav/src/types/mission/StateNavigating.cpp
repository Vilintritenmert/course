#include <cmath>

#include "MissionProcessor.hpp"
#include "StateEngaging.hpp"
#include "StateNavigating.hpp"

std::unique_ptr<IMissionState>
StateNavigating::execute(MissionProcessor &mp,
                         const MissionEngagement &engagement) {
  if (!engagement.stagingMode && mp._droneDetails->getState() == MOVING &&
      engagement.distToPrediction <=
          engagement.horizonDistance + mp._config->getHitRadius()) {
    ++mp._step;
    return std::make_unique<StateEngaging>();
  }

  Position navPos;
  if (engagement.stagingMode && engagement.distToPrediction > 1e-3f) {
    navPos = engagement.predPos +
             Position::normalize(mp._droneDetails->getPosition() -
                                 engagement.predPos) *
                 (engagement.horizonDistance + mp._config->getAccelPath());
  } else {
    navPos = engagement.predPos;
  }

  float desiredDir =
      atan2f(navPos.getY() - mp._droneDetails->getPosition().getY(),
             navPos.getX() - mp._droneDetails->getPosition().getX());

  mp._droneDetails->updateDrone(desiredDir, mp._turnAngleLeft);

  mp._timeManagement->tick();
  ++mp._step;
  mp._totalSteps = mp._step;

  return std::make_unique<StateNavigating>();
}
