#pragma once

#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

class JsonIOFailure : public std::exception {
private:
  std::string _message;
public:
  JsonIOFailure(const std::string &message) : _message(message) {}
  const char *what() const throw() { return _message.c_str(); }
};

json loadJsonFile(const std::string &path);
void storeJson(const std::string &fileName, const json &output);