#pragma once

#include "interfaces.hpp"

namespace uav {

class FileConfigLoader : public IConfigLoader {
public:
  auto load(const std::string &dataDir) -> bool override;
  auto getConfig() const -> const DroneConfig & override { return config_; }
  auto getAmmoParams() const -> const AmmoParams & override { return ammo_; }

private:
  void loadConfig(const std::string &filePath);
  void loadAmmo(const std::string &filePath);

  DroneConfig config_;
  AmmoParams ammo_;
};

} // namespace uav
