#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace uav {

struct BallisticTable {
  std::vector<float> axisZ0; // висота
  std::vector<float> axisV0; // швидкість
  std::vector<float> axisM;  // маса
  std::vector<float> axisD;  // опір
  std::vector<float> axisL;  // підйомна сила

  struct Result {
    float t = 0.F;     // час польоту
    float hDist = 0.F; // горизонтальна дистанція
  };

  // Плоский масив розміром |Z0| * |V0| * |M| * |D| * |L|.
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

  // Завантажує таблицю з текстового файлу (формат описаний в ДЗ9).
  auto load(const std::string &path) -> bool;

  // Пошук з лінійною інтерполяцією по всіх 5 осях (32 вершини гіперкуба).
  // Значення за межами таблиці - clamp до крайнього вузла.
  auto lookup(float z0, float v0, float m, float d, float l) const -> Result;
};

} // namespace uav
