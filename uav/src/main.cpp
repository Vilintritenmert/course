// ДЗ 9 (uav_ai): рефакторинг монолітного main() на набір класів з чіткими
// інтерфейсами (ITargetProvider, IBallisticSolver, IConfigLoader),
// створюваних через фабрику, і скомбінованих у MissionProcessor
// (патерн Стратегія). Логіка симуляції не змінилась порівняно з ДЗ 8.

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "uav_ai/factory.hpp"
#include "uav_ai/mission_processor.hpp"

using namespace uav;

auto main(int argc, char **argv) -> int {
  if (argc != 2) {
    std::cerr << "usage: uav_ai <data_dir>\n";
    return 1;
  }

  const std::string dataDir = argv[1];
  const std::string targetsPath = dataDir + "/targets.json";
  const std::string outputPath = dataDir + "/simulation.json";

  // Компоненти створюються через фабрику - викликаючий код працює лише з
  // інтерфейсами і не знає конкретних типів. Володіння передається у
  // MissionProcessor разом з std::move() - назовні жодних сирих вказівників.
  std::unique_ptr<IConfigLoader> loader = createLoader(LoaderType::FILE);
  std::unique_ptr<ITargetProvider> provider = createProvider(ProviderType::JSON);
  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::ANALYTICAL);

  MissionProcessor mission(std::move(loader), std::move(provider),
                           std::move(solver));

  try {
    if (!mission.init(dataDir, targetsPath)) {
      throw std::runtime_error("Error: mission initialization failed");
    }

    while (mission.hasNext()) {
      mission.step();
    }

    mission.writeOutput(outputPath);
    std::cout << "Simulation finished in " << mission.getStepCount()
              << " steps. Output: " << outputPath << '\n';
  } catch (const std::runtime_error &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return 0;
}
