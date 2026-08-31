#pragma once

#include "uav_ai/interfaces.hpp"

namespace uav {

// Читає config.json (параметри дрона/симуляції) та ammo.json (параметри
// боєприпасів) з директорії даних, обирає ammo за назвою з конфігу.
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
