#include "config_loader.hpp"
#include <exception>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ConfigLoaderOptions::ConfigLoaderOptions(std::string mainConfig,
                                         std::string ammoConfig,
                                         std::string targetConfig,
                                         std::string resultPath)
    : _config(mainConfig), _ammoParams(ammoConfig),
      _targetConfig(targetConfig), _resultPath(resultPath) {}

json JsonConfigLoader::loadFile(const std::string &path) const {
  try {
    std::ifstream configStream(path);
    if (!configStream.is_open()) {
      throw std::runtime_error("Cannot open " + path);
    }
    json jsonData;
    configStream >> jsonData;
    configStream.close();

    return jsonData;
  } catch (std::exception error) {
    throw ConfigLoaderException("Failed load config: `" + path + "`");
  }
}

void JsonConfigLoader::loadAmmoConfig(const std::string &path) {
  try {
    json jsonData = this->loadFile(path);
    std::vector<AmmoParams> ammoItems;
    ammoItems.reserve(jsonData.size());

    for (const json &ammoRawData : jsonData) {
      AmmoParams ammoItem(std::string(ammoRawData["name"].get<std::string>()),
                          static_cast<float>(ammoRawData["mass"]),
                          static_cast<float>(ammoRawData["drag"]),
                          static_cast<float>(ammoRawData["lift"]));
      ammoItems.push_back(ammoItem);
    }

    _ammoConfig = new AmmoConfig(ammoItems);
  } catch (std::exception error) {
    throw ConfigLoaderException("Failed load config: `" + path + "`");
  }
}

void JsonConfigLoader::loadDronConfig(const std::string &path) {
  try {
    json jsonData = this->loadFile(path);
    _mainConfig =
        new Config(Position(jsonData["drone"]["position"]["x"].get<float>(),
                            jsonData["drone"]["position"]["y"].get<float>()),
                   jsonData["drone"]["altitude"].get<float>(),
                   jsonData["drone"]["initialDirection"].get<float>(),
                   jsonData["drone"]["attackSpeed"].get<float>(),
                   jsonData["drone"]["accelerationPath"].get<float>(),
                   jsonData["drone"]["angularSpeed"].get<float>(),
                   jsonData["drone"]["turnThreshold"].get<float>(),
                   jsonData["simulation"]["timeStep"].get<float>(),
                   jsonData["simulation"]["hitRadius"].get<float>(),
                   jsonData["targetArrayTimeStep"].get<float>(),
                   std::string(jsonData["ammo"].get<std::string>()));
  } catch (std::exception error) {
    throw ConfigLoaderException("Failed load config: `" + path + "`");
  }
}

void JsonConfigLoader::loadConfig(
    const ConfigLoaderOptions *configLoaderOptions) {
  loadAmmoConfig(configLoaderOptions->getAmmoConfigFilePath());
  loadDronConfig(configLoaderOptions->getConfigFilePath());
  json targetsConfigJsonData =
      loadFile(configLoaderOptions->getTargetConfigFilePath());
}

AmmoConfig *JsonConfigLoader::getAmmoConfig() const { return _ammoConfig; }

Config *JsonConfigLoader::getConfig() const { return _mainConfig; }
