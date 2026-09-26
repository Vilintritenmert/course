#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "drone_physics.hpp"
#include "factory.hpp"
#include "mission_processor.hpp"
#include "thread_safe_target_provider.hpp"

using namespace uav;

auto main(int argc, char **argv) -> int {
  if (argc != 2) {
    std::cerr << "usage: uav <data_dir>\n";
    return 1;
  }

  const std::string dataDir = argv[1];
  const std::string targetsPath = dataDir + "/targets.json";
  const std::string outputPath = dataDir + "/simulation.json";

  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::ANALYTICAL);

  auto providerOwned = createProvider(ProviderType::THREADED);
  auto *provider =
      dynamic_cast<ThreadSafeTargetProvider *>(providerOwned.get());
  auto physics = std::make_unique<DronePhysics>();

  try {
    if (!loader->load(dataDir)) {
      throw std::runtime_error("Error: mission initialization failed");
    }
    const DroneConfig &config = loader->getConfig();

    if (!provider->load(targetsPath)) {
      throw std::runtime_error("Error: failed to load targets from `" +
                               targetsPath + "`");
    }
    provider->setArrayTimeStep(config.arrayTimeStep);
    provider->setTimeScale(config.timeScale);

    physics->init(config);

    MissionProcessor mission(std::move(loader), provider, physics.get(),
                             std::move(solver));
    if (!mission.init(dataDir)) {
      throw std::runtime_error("Error: mission initialization failed");
    }

    std::thread providerThread(&ThreadSafeTargetProvider::run, provider);
    std::thread physicsThread(&DronePhysics::run, physics.get());
    std::thread missionThread(&MissionProcessor::run, &mission);

    while (!provider->isThreadReady() || !physics->isThreadReady() ||
          !mission.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider->start();
    physics->start();
    mission.start();

    missionThread.join();
    physics->stop();
    provider->stop();

    providerThread.join();
    physicsThread.join();

    mission.writeOutput(outputPath);
    std::cout << "Simulation finished in " << mission.getStepCount()
              << " steps. Output: " << outputPath << '\n';
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
