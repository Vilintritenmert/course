cmake_minimum_required(VERSION 3.20)

project(uav_calc CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(uav_balistics_check src/main.cpp)

add_library(ballistics src/ballistics.cpp)

target_include_directories(ballistics PUBLIC include)

target_link_libraries(uav_balistics_check PRIVATE ballistics)

add_executable(ballistics_tests tests/ballistics_tests.cpp)

target_link_libraries(ballistics_tests PRIVATE GTest::gtest_main)
target_link_libraries(ballistics_tests PRIVATE ballistics)

gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(uav_balistics_check PRIVATE -Wall -Wextra -pedantic)

enable_testing()
