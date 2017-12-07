project (Kalman)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/LatLong-UTMconversion.cpp
   src/main.cpp
   src/kalman_ctrv.cpp)

add_executable(kalman_ctrv ${sources})
