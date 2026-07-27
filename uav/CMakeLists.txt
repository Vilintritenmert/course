cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 06. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(uav_checker CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
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

add_executable(uav_checker src/main.cpp)
add_library(ballistics_lib 
    src/config_loader.cpp
    src/json_helper.cpp
    src/types.cpp
)

target_include_directories(ballistics_lib PUBLIC include)

target_link_libraries(uav_checker PRIVATE ballistics_lib)
target_link_libraries(uav_checker PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(ballistics_lib PRIVATE nlohmann_json::nlohmann_json)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(uav_checker PRIVATE -Wall -Wextra -pedantic)