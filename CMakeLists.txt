cmake_minimum_required(VERSION 3.27)
project(pathfinding_astar)

set(CMAKE_CXX_STANDARD 17)

add_executable(pathfinding_astar main.cpp
        source/astar.h)
