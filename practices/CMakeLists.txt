cmake_minimum_required(VERSION 3.20)

project(practices CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
)
FetchContent_MakeAvailable(nlohmann_json)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_executable(practices src/main.cpp)

target_compile_options(practices PRIVATE -Wall -Wextra -pedantic)