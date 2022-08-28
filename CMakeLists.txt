cmake_minimum_required(VERSION 2.8)
project(trajectory_generator)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-O3")

# include Eigen
include_directories("/usr/include/eigen3")

# project header
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(trajectory_generator ${PROJECT_SOURCE_DIR}/src/trajectory_generator.cpp)

add_library(ligTrajectoryGenerator SHARED 
                ${PROJECT_SOURCE_DIR}/src/python_wrapper.cpp
                ${PROJECT_SOURCE_DIR}/src/trajectory_generator.cpp
)


                

