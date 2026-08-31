#include "uav_ai/mission_processor.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "uav_ai/drone_state.hpp"

using json = nlohmann::json;

namespace uav {

MissionProcessor::MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                                   std::unique_ptr<ITargetProvider> provider,
                                   std::unique_ptr<IBallisticSolver> solver)
    : configLoader_(std::move(configLoader)), provider_(std::move(provider)),
      solver_(std::move(solver)) {}

MissionProcessor::~MissionProcessor() = default;

auto MissionProcessor::init(const std::string &dataDir,
                            const std::string &targetsPath) -> bool {
  initialized_ = false;

  if (!configLoader_->load(dataDir)) {
    throw std::runtime_error("Error: failed to load config from `" + dataDir +
                             "`");
  }
  config_ = configLoader_->getConfig();
  ammo_ = configLoader_->getAmmoParams();

  if (!provider_->load(targetsPath)) {
    throw std::runtime_error("Error: failed to load targets from `" +
                             targetsPath + "`");
  }
  provider_->setArrayTimeStep(config_.arrayTimeStep);
  provider_->reset();

  if (!solver_->init(ammo_, config_.attackSpeed, config_.altitude,
                     config_.accelPath)) {
    throw std::runtime_error("Error: failed to initialize ballistic solver");
  }

  resetDrone();
  history_.clear();
  currentTime_ = 0.F;
  selectedTarget_ = -1;
  stepCount_ = 0;
  finished_ = false;
  initialized_ = true;
  return true;
}

void MissionProcessor::resetDrone() {
  drone_ = Drone();
  drone_.pos = config_.startPos;
  drone_.dir = config_.initialDir;
  drone_.speed = 0.F;
  state_ = std::make_unique<StateStopped>();
}

auto MissionProcessor::hasNext() const -> bool {
  return initialized_ && !finished_ && stepCount_ < MAX_STEPS;
}

void MissionProcessor::step() {
  if (!hasNext()) {
    return;
  }

  const int targetCount = provider_->getTargetCount();
  if (targetCount <= 0) {
    finished_ = true;
    return;
  }

  const float acceleration =
      (config_.attackSpeed * config_.attackSpeed) / (2.F * config_.accelPath);

  DroneContext ctx{drone_.pos,          drone_.dir,     drone_.speed,
                    drone_.turnGoalDir, drone_.turnRemaining,
                    drone_.turnSign,    0.F,            acceleration,
                    config_};

  std::vector<TargetCandidate> candidates(targetCount);
  for (int i = 0; i < targetCount; ++i) {
    candidates[i] = solver_->solve(drone_.pos, provider_->getTarget(i));
  }

  int bestTarget = 0;
  float bestEffectiveTime = std::numeric_limits<float>::max();
  for (int i = 0; i < targetCount; ++i) {
    float effectiveTime = candidates[i].totalTime;
    if (i != selectedTarget_) {
      effectiveTime += state_->timeToStop(ctx);
    }
    if (effectiveTime < bestEffectiveTime) {
      bestEffectiveTime = effectiveTime;
      bestTarget = i;
    }
  }
  selectedTarget_ = bestTarget;

  const TargetCandidate &chosen = candidates[selectedTarget_];
  const float dt = config_.simTimeStep;

  ctx.desiredDir = chosen.heading;
  auto next = state_->execute(ctx);
  if (next) {
    state_ = std::move(next);
  }

  SimStep s;
  s.pos = drone_.pos;
  s.direction = drone_.dir;
  s.state = state_->name();
  s.targetIdx = selectedTarget_;
  s.dropPoint = chosen.releasePoint;
  const Coord aimDir = {std::cos(drone_.dir), std::sin(drone_.dir)};
  s.aimPoint = drone_.pos + aimDir * solver_->getHorizontalDistance();
  s.predictedTarget = chosen.predictedTarget;
  history_.push_back(s);
  ++stepCount_;

  provider_->advance(dt);
  currentTime_ += dt;

  const float distToRelease = length(chosen.releasePoint - drone_.pos);
  if (distToRelease <= config_.hitRadius || stepCount_ >= MAX_STEPS) {
    finished_ = true;
  }
}

void MissionProcessor::reset() {
  resetDrone();
  provider_->reset();
  history_.clear();
  currentTime_ = 0.F;
  selectedTarget_ = -1;
  stepCount_ = 0;
  finished_ = false;
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver) {
  solver_ = std::move(solver);
  solver_->init(ammo_, config_.attackSpeed, config_.altitude, config_.accelPath);
}

void MissionProcessor::writeOutput(const std::string &outputPath) const {
  json out;
  out["totalSteps"] = static_cast<int>(history_.size());
  out["steps"] = json::array();

  for (const auto &s : history_) {
    json step;
    step["position"] = {{"x", s.pos.x}, {"y", s.pos.y}};
    step["direction"] = s.direction;
    step["state"] = s.state;
    step["targetIndex"] = s.targetIdx;
    step["dropPoint"] = {{"x", s.dropPoint.x}, {"y", s.dropPoint.y}};
    step["aimPoint"] = {{"x", s.aimPoint.x}, {"y", s.aimPoint.y}};
    step["predictedTarget"] = {{"x", s.predictedTarget.x},
                               {"y", s.predictedTarget.y}};
    out["steps"].push_back(step);
  }

  std::ofstream outFile(outputPath);
  if (!outFile.is_open()) {
    throw std::runtime_error("Error: cannot open `" + outputPath +
                             "` for writing");
  }
  outFile << out.dump(2);
}

} // namespace uav
