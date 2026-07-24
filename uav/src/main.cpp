#include <iostream>
#include <string>

#include "ballistic_solver.hpp"
#include "config_loader.hpp"
#include "factory.hpp"
#include "mission_planner.hpp"
#include "target_provider.hpp"
#include "types.hpp"

int main(int argc, char **argv) {
  try {
    if (argc != 2) {
      throw std::invalid_argument("usage: uav_calculator <data_folder_path>\n");
    }

    const std::string dataFolderName = argv[1];
    const ConfigLoaderOptions configLoaderOptions{
        dataFolderName + std::string("/config.json"),
        dataFolderName + std::string("/ammo.json"),
        dataFolderName + std::string("/targets.json")};

    Factory factory(configLoaderOptions);

    IConfigLoader *configLoader = factory.createLoader(LoaderType::FILE);

    const Config *mainCfg = configLoader->getConfig();
    const AmmoConfig *ammoConfigObj = configLoader->getAmmoConfig();

    TimeManagement *timeManagement = factory.getTimeManagement();

    DroneDetails droneDetails(mainCfg->getStartPos(),
                              *ammoConfigObj->findAmmo(mainCfg->getAmmoName()),
                              mainCfg->getAltitude(), mainCfg->getInitialDir(),
                              mainCfg->getAttackSpeed(),
                              mainCfg->getAccelPath(),
                              mainCfg->getAngularSpeed());

    ITargetProvider *targetProvider =
        factory.createProvider(ProviderType::JSON);

    IBallisticSolver *ballisticSolver =
        factory.createSolver(SolverType::ANALYTICAL);
    MissionPlanner planner(droneDetails, ballisticSolver, targetProvider,
                           timeManagement, mainCfg, dataFolderName);

    return planner.runSimulation();
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}