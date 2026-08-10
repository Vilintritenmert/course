#pragma once

#include <nlohmann/json.hpp>

#include "IConfigLoader.hpp"

class JsonConfigLoader : public IConfigLoader {
private:
  std::shared_ptr<AmmoConfig> _ammoConfig;
  std::shared_ptr<Config> _mainConfig;

  nlohmann::json loadFile(const std::string &path) const;
  void loadAmmoConfig(const std::string &path);
  void loadDronConfig(const std::string &path);

public:
  void loadConfig(const std::shared_ptr<ConfigLoaderOptions> configLoaderOptions) override;
  std::shared_ptr<Config> getConfig() const override;
  std::shared_ptr<AmmoConfig> getAmmoConfig() const override;
};
