#include "ballistics.hpp"
#include <iostream>
#include <fstream>
#include <istream>
#include <span>
#include <stdexcept>
#include <ostream>

using namespace std;

struct EnsureNextVariableExist {};
const EnsureNextVariableExist ensure_next_variable_is_exist;

auto operator>>(istream& is, EnsureNextVariableExist) -> istream&
{
  if (is.eof()) {
    throw invalid_argument("Input file is incorrect");
  }

  return is;
}

auto operator>>(istream& is, BallisticsInput& input) -> istream&
{
  return is >> input.initial_dron.x >> ensure_next_variable_is_exist >> input.initial_dron.y >> ensure_next_variable_is_exist >>
         input.initial_dron.z >> ensure_next_variable_is_exist >> input.target.x >> ensure_next_variable_is_exist >> input.target.y >>
         ensure_next_variable_is_exist >> input.attack_speed >> ensure_next_variable_is_exist >> input.acceleration_path >>
         ensure_next_variable_is_exist >> input.ammo_name;
}

auto readInput(const string& filePath) -> BallisticsInput    
{
  BallisticsInput input;

  fstream input_file(filePath);
  if (!input_file.is_open()) {
    throw invalid_argument("`" + filePath + "` file not found");
  }
  input_file >> input;
  input_file.close();

  return input;
}

void writeOutput(const string& filePath, const DropSolution& drop_solution)
{
  ofstream output_file(filePath);
  if (!output_file.is_open()) {
    throw invalid_argument("`" + filePath + "` file is not available");
  }

  if (drop_solution.tmp_x != 0 && drop_solution.tmp_y != 0) {
    output_file << drop_solution.tmp_x << " " << drop_solution.tmp_y << " ";
  }

  output_file << drop_solution.fire_x << " " << drop_solution.fire_y << '\n';

  output_file.close();
}

auto main(int argc, char** argv) -> int
{
  try {
    if (argc != 2) {
      throw invalid_argument("usage: uav_balistics_check <input_path>\n");
    }

    std::span<char*> args{argv, static_cast<size_t>(argc)};

    BallisticsInput input = readInput(string(args[1]) + "/input.txt");

    DropSolution drop_solution = computeDropSolution(input);

    writeOutput(string(args[1]) + "/output.txt", drop_solution);

    return 0;
  }
  catch (const invalid_argument& e) {
    cerr << "Calculation failed with the reason: " << e.what() << '\n';

    return 1;
  }
}