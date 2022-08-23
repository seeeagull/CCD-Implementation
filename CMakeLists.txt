cmake_minimum_required(VERSION 3.23)
project(CCD)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fopenmp")
endif()

include_directories(~/libigl/include)

add_executable(CCD main.cpp src/calculator.cpp src/Aabb.cpp include/Aabb.h include/calculator.h src/constructor.cpp include/constructor.h)
