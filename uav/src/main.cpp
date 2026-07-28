#include <iostream>
#include <string>

#include "IConfigLoader.hpp"
#include "MissionProcessor.hpp"

int main(int argc, char **argv) {
  try {
    if (argc != 2) {
      throw std::invalid_argument("usage: uav_calculator <data_folder_path>\n");
    }

    const std::string dataFolderName = argv[1];
    const ConfigLoaderOptions configLoaderOptions{
        dataFolderName + std::string("/config.json"),
        dataFolderName + std::string("/ammo.json"),
        dataFolderName + std::string("/targets.json"),
        dataFolderName + std::string("/simulation.json")};

    MissionProcessor processor(configLoaderOptions);

    while (processor.hasNext()) {
      processor.step();
    }

    processor.storeSimulation();
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}