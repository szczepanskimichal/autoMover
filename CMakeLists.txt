cmake_minimum_required(VERSION 3.20)

project(autoMower)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(autoMower
    src/main.cpp
    src/DriveController.cpp
    src/SimulationDriver.cpp
)

target_include_directories(autoMower PRIVATE include)