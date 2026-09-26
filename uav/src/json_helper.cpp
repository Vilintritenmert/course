#include "json_helper.hpp"

#include <fstream>

json loadJsonFile(const std::string &path) {
  std::ifstream fc(path);
  if (!fc.is_open()) {
    throw JsonIOFailure("Cannot open " + path);
  }
  json jc;
  fc >> jc;
  fc.close();

  return jc;
}

void storeJson(const std::string &fileName, const json &output) {
  std::ofstream fout(fileName);
  if (!fout.is_open()) {
    throw JsonIOFailure("Cannot write to " + fileName);
  } else {
    fout << output.dump(2);
    fout.close();
  }
}
