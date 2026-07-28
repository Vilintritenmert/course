#pragma once

#include <nlohmann/json.hpp>

#include "IConfigLoader.hpp"

class JsonConfigLoader : public IConfigLoader {
private:
  AmmoConfig *_ammoConfig;
  Config *_mainConfig;

  nlohmann::json loadFile(const std::string &path) const;
  void loadAmmoConfig(const std::string &path);
  void loadDronConfig(const std::string &path);

public:
  void loadConfig(const ConfigLoaderOptions *configLoaderOptions) override;
  Config *getConfig() const override;
  AmmoConfig *getAmmoConfig() const override;
};
