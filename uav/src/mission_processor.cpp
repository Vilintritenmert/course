#include "mission_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <thread>

#include <nlohmann/json.hpp>

#include "drone_physics.hpp"
#include "drone_state.hpp"

using json = nlohmann::json;

namespace uav {

MissionProcessor::MissionProcessor(std::unique_ptr<IConfigLoader> configLoader,
                                   ITargetProvider *provider,
                                   DronePhysics *physics,
                                   std::unique_ptr<IBallisticSolver> solver)
    : configLoader_(std::move(configLoader)), provider_(provider),
      physics_(physics), solver_(std::move(solver)) {}

MissionProcessor::~MissionProcessor() = default;

auto MissionProcessor::init(const std::string &dataDir) -> bool {
  initialized_ = false;

  if (!configLoader_->load(dataDir)) {
    throw std::runtime_error("Error: failed to load config from `" + dataDir +
                             "`");
  }
  config_ = configLoader_->getConfig();
  ammo_ = configLoader_->getAmmoParams();

  if (!solver_->init(ammo_, config_.attackSpeed, config_.altitude,
                     config_.accelPath)) {
    throw std::runtime_error("Error: failed to initialize ballistic solver");
  }

  state_ = std::make_unique<StateStopped>();
  history_.clear();
  currentTime_ = 0.F;
  selectedTarget_ = -1;
  stepCount_ = 0;
  finished_ = false;
  initialized_ = true;
  return true;
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

  const DroneTelemetry telemetry = physics_->getTelemetry();

  const float acceleration =
      (config_.attackSpeed * config_.attackSpeed) / (2.F * config_.accelPath);

  DroneContext ctx{telemetry.pos, telemetry.direction,
                    length(telemetry.speed), 0.F, acceleration, config_, {}};

  std::vector<TargetCandidate> candidates(targetCount);
  for (int i = 0; i < targetCount; ++i) {
    candidates[i] = solver_->solve(telemetry.pos, provider_->getTarget(i));
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

  ctx.desiredDir = chosen.heading;
  auto next = state_->execute(ctx);
  physics_->sendCommand(ctx.command);
  if (next) {
    state_ = std::move(next);
  }

  SimStep s;
  s.pos = telemetry.pos;
  s.direction = telemetry.direction;
  s.state = droneModeName(telemetry.mode);
  s.targetIdx = selectedTarget_;
  s.dropPoint = chosen.releasePoint;
  const Coord aimDir = {std::cos(telemetry.direction), std::sin(telemetry.direction)};
  s.aimPoint = telemetry.pos + aimDir * solver_->getHorizontalDistance();
  s.predictedTarget = chosen.predictedTarget;
  s.timeSecSinceStart = telemetry.timeSecSinceStart;
  history_.push_back(s);
  ++stepCount_;

  currentTime_ += config_.simTimeStep;

  const float distToRelease = length(chosen.releasePoint - telemetry.pos);
  if (distToRelease <= config_.hitRadius || stepCount_ >= MAX_STEPS) {
    finished_ = true;
  }
}

void MissionProcessor::reset() {
  state_ = std::make_unique<StateStopped>();
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

void MissionProcessor::run() {
  ready_.store(true);
  while (!running_.load() && !stopRequested_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const float dt = config_.simTimeStep;
  while (!stopRequested_.load() && hasNext()) {
    step();
    std::this_thread::sleep_for(
        std::chrono::duration<float>(dt / config_.timeScale));
  }
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
    step["timeSecSinceStart"] = s.timeSecSinceStart;
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
