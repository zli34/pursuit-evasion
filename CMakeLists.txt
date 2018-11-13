cmake_minimum_required(VERSION 3.1)
project(coopPE)

include(CTest)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

add_subdirectory(libs/googletest)
set(NJM_CPP_FORCE_BUILD_TEST YES)
add_subdirectory(libs/njm_cpp)
add_subdirectory(src/main)
add_subdirectory(src/bin)
add_subdirectory(test)
