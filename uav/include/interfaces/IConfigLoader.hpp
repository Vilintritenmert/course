#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "Position.hpp"
#include "AmmoParams.hpp"

class AmmoConfig {
private:
  std::vector<AmmoParams> _ammoConfig;

public:
  AmmoConfig(std::vector<AmmoParams> ammoParams) : _ammoConfig(ammoParams) {}

  AmmoParams *findAmmo(std::string name) const {
    for (const AmmoParams &param : _ammoConfig) {
      if (name == param.getName()) {
        return const_cast<AmmoParams *>(&param);
      }
    }

    return nullptr;
  }
};

class Config {
private:
  Position _startPos;
  float _altitude, _initialDir, _attackSpeed, _accelPath;
  float _arrayTimeStep, _simTimeStep, _hitRadius, _angularSpeed, _turnThreshold;
  std::string _ammoName;

public:
  Config(Position startPos, float altitude, float initialDir, float attackSpeed,
         float accelPath, float angularSpeed, float turnThreshold,
         float simTimeStep, float hitRadius, float arrayTimeStep,
         const std::string ammoName)
      : _startPos(startPos), _altitude(altitude), _initialDir(initialDir),
        _attackSpeed(attackSpeed), _accelPath(accelPath),
        _arrayTimeStep(arrayTimeStep), _simTimeStep(simTimeStep),
        _hitRadius(hitRadius), _angularSpeed(angularSpeed),
        _turnThreshold(turnThreshold), _ammoName(ammoName) {
    validateParameters();
  }

  void validateParameters() const {
    if (_simTimeStep <= 0.0f || _arrayTimeStep <= 0.0f) {
      throw std::runtime_error("ERROR: Invalid parameters\n");
    }
  }

  Position getStartPos() const { return _startPos; }
  float getAltitude() const { return _altitude; }
  float getInitialDir() const { return _initialDir; }
  float getAttackSpeed() const { return _attackSpeed; }
  float getAccelPath() const { return _accelPath; }
  const std::string getAmmoName() const { return _ammoName; }
  float getArrayTimeStep() const { return _arrayTimeStep; }
  float getSimTimeStep() const { return _simTimeStep; }
  float getHitRadius() const { return _hitRadius; }
  float getAngularSpeed() const { return _angularSpeed; }
  float getTurnThreshold() const { return _turnThreshold; }
  float getAngelStep() const { return getAngularSpeed() * getSimTimeStep();}
};

class ConfigLoaderException : public std::exception {
private:
  std::string _message;

public:
  ConfigLoaderException(const std::string &message) : _message(message) {}
  const char *what() const throw() { return _message.c_str(); }
};

class ConfigLoaderOptions {
private:
  std::string _config;
  std::string _ammoParams;
  std::string _targetConfig;
  std::string _ballistikTable;
  std::string _resultPath;

public:
  ConfigLoaderOptions(std::string mainConfig, std::string ammoConfig,
                      std::string targetConfig, std::string resultPath,
                      std::string ballistikTable = "");
  std::string getConfigFilePath() const { return _config; };
  std::string getAmmoConfigFilePath() const { return _ammoParams; };
  std::string getTargetConfigFilePath() const { return _targetConfig; };
  std::string getBallistikTable() const { return _ballistikTable; };
  std::string getResultPath() const { return _resultPath; };
};

class IConfigLoader {
public:
  virtual void loadConfig(const std::shared_ptr<ConfigLoaderOptions> configLoaderOptions) = 0;
  virtual std::shared_ptr<Config> getConfig() const = 0;
  virtual std::shared_ptr<AmmoConfig> getAmmoConfig() const = 0;
  virtual ~IConfigLoader() = default;
};
