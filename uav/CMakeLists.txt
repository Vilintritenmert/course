cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 09 (uav). Його можна збирати
# як через root CMakeLists.txt, так і напряму з цієї директорії.
project(uav CXX)

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(uav
    src/main.cpp
    src/file_config_loader.cpp
    src/json_target_provider.cpp
    src/analytical_solver.cpp
    src/table_solver.cpp
    src/ballistic_table.cpp
    src/drone_state.cpp
    src/factory.cpp
    src/mission_processor.cpp
)
target_include_directories(uav PRIVATE include)
target_link_libraries(uav PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(uav PRIVATE -Wall -Wextra -pedantic)
