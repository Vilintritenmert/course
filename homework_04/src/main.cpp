#include <iostream>
#include <ostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>

using namespace std;

fstream getStream(const char* file_name)
{
  fstream stream_file(file_name);
  if (!stream_file.is_open()) {
    throw invalid_argument("`" + string(file_name) + "` file is not available");
  }

  return stream_file;
}

const float ticks_per_revolution = 1024;
const float wheel_radius_m = 0.3;
const float wheelbase_m = 1.0;

struct WheelImpulse {
  float timestamp = 0;
  float fl_ticks = 0;
  float fr_ticks = 0;
  float bl_ticks = 0;
  float br_ticks = 0;

  WheelImpulse operator-(const WheelImpulse& other)
  {
    return WheelImpulse{
      other.timestamp - timestamp,
      other.fl_ticks - fl_ticks,
      other.fr_ticks - fr_ticks,
      other.bl_ticks - bl_ticks,
      other.br_ticks - br_ticks,
    };
  }
};

struct Nrk {
  float x = 0;
  float y = 0;
  float theta = 0;

  WheelImpulse last_impulse;

  Nrk& operator+=(const WheelImpulse& new_impulse)
  {
    const WheelImpulse impulse_difference = last_impulse - new_impulse;

    const float d_left = (impulse_difference.fl_ticks + impulse_difference.bl_ticks) / 2;
    const float d_right = (impulse_difference.fr_ticks + impulse_difference.br_ticks) / 2;

    const float distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

    const float d_l = d_left * distance_per_tick;
    const float d_r = d_right * distance_per_tick;

    const float distance = (d_l + d_r) / 2;
    const float d_theta = (d_r - d_l) / wheelbase_m;

    x += distance * cos(theta + d_theta / 2);
    y += distance * sin(theta + d_theta / 2);

    theta += d_theta;

    last_impulse = new_impulse;

    return *this;
  }
};

struct EnsureNextParameterExist {};

istream& operator>>(istream& is, EnsureNextParameterExist)
{
  if (is.eof()) {
    throw invalid_argument("Input file is incorrect");
  }

  return is;
}

istringstream& operator>> (istringstream& iss, EnsureNextParameterExist validateInput);
istream& operator>>(istream& is, WheelImpulse& impulse)
{
  EnsureNextParameterExist validateInput;

  is >> impulse.timestamp >> validateInput 
     >> impulse.fl_ticks >> validateInput 
     >> impulse.fr_ticks >> validateInput 
     >> impulse.bl_ticks >> validateInput 
     >> impulse.br_ticks;

  return is;
}


ostream& operator<<(ostream& os, const Nrk& nrk)
{
  os 
  << nrk.last_impulse.timestamp << " " 
  << nrk.x << " " 
  << nrk.y << " "
  << nrk.theta;

  return os;
}

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw invalid_argument("usage: ugv_odometry <input_path>\n");
    }

    fstream input_file = getStream(argv[1]);
    WheelImpulse new_impulse;
    Nrk nrk;
    string line;

    while (getline(input_file, line)) {
      if (line.empty() || input_file.eof()) {
        break;
      }

      istringstream line_stream(line);
      line_stream >> new_impulse;

      nrk += new_impulse;

      if (nrk.last_impulse.timestamp == 0) {
        continue;
      }
      cout << nrk << endl;
    }

    input_file.close();

    return 0;
  }
  catch (std::invalid_argument e) {
    std::cerr << "Something went wrong: " << e.what() << endl;
    return 1;
  }
}
