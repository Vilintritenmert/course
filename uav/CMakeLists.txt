cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 06. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(uav_calculator CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GoogleTest)
include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(googletest nlohmann_json)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(uav_calculator src/main.cpp)

target_link_libraries(uav_calculator PRIVATE nlohmann_json::nlohmann_json)


# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(uav_calculator PRIVATE -Wall -Wextra -pedantic)
