#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace uav {

struct BallisticTable {
  std::vector<float> axisZ0;
  std::vector<float> axisV0;
  std::vector<float> axisM;
  std::vector<float> axisD;
  std::vector<float> axisL;

  struct Result {
    float t = 0.F;
    float hDist = 0.F;
  };

  std::vector<Result> data;

  auto index(int iz, int iv, int im, int id, int il) const -> size_t {
    return ((((static_cast<size_t>(iz) * axisV0.size() + iv) * axisM.size() +
              im) *
                 axisD.size() +
             id) *
                axisL.size() +
            il);
  }

  auto at(int iz, int iv, int im, int id, int il) const -> const Result & {
    return data[index(iz, iv, im, id, il)];
  }

  auto load(const std::string &path) -> bool;

  auto lookup(float z0, float v0, float m, float d, float l) const -> Result;
};

}
