#include "telemetry.hpp"

#include <iostream>
#include <ostream>
#include <stdexcept>

int main(int argc, char** argv)
{
  try {
    if (argc != 2) {
      throw std::invalid_argument("usage: telemetry_check <input_path>");
    }

    Frame frames[MAX_TELEMETRY_FRAMES];
    const int frame_count = read_frames(argv[1], frames, MAX_TELEMETRY_FRAMES);

    if (frame_count == 0) {
      throw std::invalid_argument("error: input file is empty");
    } 

    const Summary summary = summarize(frames, frame_count);
    print_summary(summary);

    return 0;
  } catch(std::invalid_argument error) {
    std::cerr << "Bad input: " << error.what() << std::endl;
    
    return 1;
  }
}
