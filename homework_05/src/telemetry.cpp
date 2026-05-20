#include "telemetry.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

struct telemetry_parsing_exception : public std::invalid_argument {
  using std::invalid_argument::invalid_argument;
};

int split_line(char line[], char* fields[], int max_fields)
{
  int count = 0;
  char* cursor = line;

  while (*cursor != '\0' && count < max_fields) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
      *cursor = '\0';
      ++cursor;
    }

    if (*cursor == '\0') {
      break;
    }

    fields[count] = cursor;
    ++count;

    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' && *cursor != '\r') {
      ++cursor;
    }
  }

  if (count != EXPECTED_FIELD_COUNT) {
    throw telemetry_parsing_exception(std::string("expected ") + std::to_string(EXPECTED_FIELD_COUNT) + " fields, but got " +
                                      std::to_string(count));
  }

  return count;
}

long parse_long(const char* text)
{
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);

  if (end == text) {
    throw telemetry_parsing_exception(std::string("failed to parse long integer: ") + text);
  }

  return value;
}

int parse_int(const char* text)
{
  return static_cast<int>(parse_long(text));
}

double parse_double(const char* text)
{
  char* end = nullptr;
  const double value = std::strtod(text, &end);

  if (end == text) {
    throw telemetry_parsing_exception(std::string("failed to parse double: ") + text);
  }

  return value;
}

Frame parse_frame(char line[])
{
  char* fields[EXPECTED_FIELD_COUNT] = {};
  const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);
  (void)field_count;

  Frame frame{};
  frame.timestamp_ms = parse_long(fields[0]);
  frame.seq = parse_int(fields[1]);
  frame.voltage_v = parse_double(fields[2]);
  frame.current_a = parse_double(fields[3]);
  frame.temperature_c = parse_double(fields[4]);
  frame.gps_fix = parse_int(fields[5]);
  frame.satellites = parse_int(fields[6]);

  return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count)
{
  const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

  return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

void validateFrames(const Frame frameA, const Frame frameB) {
    const long delta_ms = frameB.timestamp_ms - frameA.timestamp_ms;
    
    if (delta_ms <= 0) {
        throw telemetry_parsing_exception(std::string("non-positive time delta between frames: ") + std::to_string(delta_ms) + " ms");
    }

    const int delta_seq = frameB.seq - frameA.seq;
    if (delta_seq != 1) {
        throw telemetry_parsing_exception(std::string("the field seq should increase by 1, but got : ") + std::to_string(delta_seq));
    }

    if (frameB.voltage_v <= 0) {
        throw telemetry_parsing_exception(std::string("voltage value should be positive, but got: ") + std::to_string(frameB.voltage_v));
    }

    if (frameB.temperature_c < -40 || frameB.temperature_c > 120) {
        throw telemetry_parsing_exception(std::string("temperature value is out of expected range (-40 to 120 °C), but got: ") + std::to_string(frameB.temperature_c));
    }

    if (frameB.gps_fix < 0 || frameB.gps_fix > 1) {
        throw telemetry_parsing_exception(std::string("gps_fix value should be 0 or 1, but got: ") + std::to_string(frameB.gps_fix));
    }

    if (frameB.satellites < 0) {
        throw telemetry_parsing_exception(std::string("satellites value should be non-negative, but got: ") + std::to_string(frameB.satellites));
    }
}

int read_frames(const char* path, Frame frames[], int max_frames)
{
  std::ifstream input{path};
  if (!input.is_open()) {
    throw std::invalid_argument(std::string("failed to open input file: ") + path);
  }

  int frame_count = 0;
  char line[MAX_LINE_LENGTH];

  while (input.getline(line, MAX_LINE_LENGTH)) {
    try {
      if (line[0] == '\0') {
        continue;
      }

      if (frame_count < max_frames) {
        frames[frame_count] = parse_frame(line);
        ++frame_count;
      }

      if (frame_count > 1) {
        validateFrames(frames[frame_count - 2], frames[frame_count - 1]);
      }

    }
    catch (telemetry_parsing_exception& error) {
      throw std::invalid_argument(std::string("invalid frame at line ") + std::to_string(frame_count + 1) + ": " + error.what());
    }
  }

  return frame_count;
}

Summary summarize(const Frame frames[], int frame_count)
{
  Summary summary{};
  summary.frames_total = frame_count;
  summary.frames_valid = frame_count;
  summary.voltage_min = frames[0].voltage_v;
  summary.voltage_max = frames[0].voltage_v;
  summary.low_voltage_frames = 0;

  double temperature_sum = 0.0;

  for (int i = 0; i < frame_count; ++i) {
    if (frames[i].voltage_v < summary.voltage_min) {
      summary.voltage_min = frames[i].voltage_v;
    }

    if (frames[i].voltage_v > summary.voltage_max) {
      summary.voltage_max = frames[i].voltage_v;
    }

    temperature_sum += frames[i].temperature_c;

    if (frames[i].voltage_v < 22.0) {
      ++summary.low_voltage_frames;
    }
  }

  const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
  summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
  summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
  return summary;
}

void print_summary(const Summary& summary)
{
  std::cout << "frames_total " << summary.frames_total << '\n';
  std::cout << "frames_valid " << summary.frames_valid << '\n';
  std::cout << "voltage_min " << summary.voltage_min << '\n';
  std::cout << "voltage_max " << summary.voltage_max << '\n';
  std::cout << "temperature_avg " << summary.temperature_avg << '\n';
  std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
  std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
