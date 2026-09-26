#include "MissionProcessor.hpp"
#include "StateEngaging.hpp"
#include "StateNavigating.hpp"

std::unique_ptr<IMissionState>
StateEngaging::execute(MissionProcessor &mp,
                       const MissionEngagement &engagement) {
  if (!engagement.stagingMode && mp._droneDetails->getState() == MOVING &&
      engagement.distToPrediction <=
          engagement.horizonDistance + mp._config->getHitRadius()) {
    ++mp._step;
    return std::make_unique<StateEngaging>();
  }

  auto next = std::make_unique<StateNavigating>();
  return next->execute(mp, engagement);
}
