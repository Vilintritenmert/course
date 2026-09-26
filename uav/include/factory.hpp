#pragma once

#include "ballistic_solver.hpp"
#include "target_provider.hpp"
#include "config_loader.hpp"
#include "types.hpp"

class FactoryLoaderException : public std::exception {
private:
  std::string _message;

public:
  FactoryLoaderException(const std::string &message) : _message(message) {}
  const char *what() const throw() { return _message.c_str(); }
};

enum class SolverType  { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE }; 

class Factory {
    ConfigLoaderOptions _options;
    TimeManagement* _timeManagement;
    IConfigLoader* _configLoader;

public:
    Factory(ConfigLoaderOptions options): _options(options) {
        initDefaultData();
    };

    void initDefaultData() {
        _configLoader = createLoader(LoaderType::FILE);
        _configLoader->loadConfig(&_options);
        _timeManagement = new TimeManagement(_configLoader->getConfig()->getSimTimeStep(), _configLoader->getConfig()->getArrayTimeStep());
    };

    IBallisticSolver* createSolver(SolverType type) {
        if (type == SolverType::ANALYTICAL) {
            return new AnalyticalSolver();
        }

        throw FactoryLoaderException("Unknown solver type");
    }; 
    ITargetProvider* createProvider(ProviderType type) {
        if (type == ProviderType::JSON) {
            return new JSONTargetProvider(_timeManagement, _options.getTargetConfigFilePath());
        }

        throw FactoryLoaderException("Unknown provider type");
    };

    IConfigLoader* createLoader(LoaderType type) {
        if (type == LoaderType::FILE) {
            return new JsonConfigLoader();
        }

        throw FactoryLoaderException("Unknown loader type");
    }; 

    TimeManagement *getTimeManagement() const {
        return _timeManagement;
    };

    IConfigLoader *getConfigLoader() const {
        return _configLoader;
    };

    ~Factory() {
        delete _timeManagement;
        delete _configLoader;
    };
};