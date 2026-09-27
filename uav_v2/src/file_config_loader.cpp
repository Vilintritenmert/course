#include "uav_v2/file_config_loader.hpp"

#include "uav_v2/uav_exception.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace uav
{

  void FileConfigLoader::load(const std::string &dataDir)
  {
    loadConfig(dataDir + "/config.json");
    loadAmmo(dataDir + "/ammo.json");
  }

  void FileConfigLoader::loadConfig(const std::string &filePath)
  {
    std::ifstream file(filePath);
    if (!file.is_open())
    {
      throw UavException("Error: cannot open `" + filePath + "`");
    }

    json j;
    file >> j;

    config_.startPos.x = j["drone"]["position"]["x"];
    config_.startPos.y = j["drone"]["position"]["y"];
    config_.altitude = j["drone"]["altitude"];
    config_.initialDir = j["drone"]["initialDirection"];
    config_.attackSpeed = j["drone"]["attackSpeed"];
    config_.accelPath = j["drone"]["accelerationPath"];
    config_.angularSpeed = j["drone"]["angularSpeed"];
    config_.turnThreshold = j["drone"]["turnThreshold"];

    config_.ammoName = j["ammo"].get<std::string>();

    config_.simTimeStep = j["simulation"]["timeStep"];
    config_.hitRadius = j["simulation"]["hitRadius"];
    config_.arrayTimeStep = j["targetArrayTimeStep"];

    if (config_.attackSpeed <= 0.F || config_.accelPath <= 0.F ||
        config_.arrayTimeStep <= 0.F || config_.simTimeStep <= 0.F ||
        config_.angularSpeed <= 0.F)
    {
      throw UavException("Error: `" + filePath +
                         "` has invalid (non-positive) parameters");
    }
  }

  void FileConfigLoader::loadAmmo(const std::string &filePath)
  {
    std::ifstream file(filePath);
    if (!file.is_open())
    {
      throw UavException("Error: cannot open `" + filePath + "`");
    }

    json j;
    file >> j;

    for (const auto &entry : j)
    {
      if (entry["name"].get<std::string>() != config_.ammoName)
      {
        continue;
      }
      ammo_.name = entry["name"].get<std::string>();
      ammo_.mass = entry["mass"];
      ammo_.drag = entry["drag"];
      ammo_.lift = entry["lift"];
      return;
    }

    throw UavException("Error: unknown ammo type `" + config_.ammoName +
                       "`");
  }

}
