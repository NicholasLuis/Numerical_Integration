# This is the CMakeLists for the Numerical Integration project
cmake_minimum_required(VERSION 3.14.0)
set(CMAKE_CXX_STANDARD 20)
project(NINT VERSION 0.1.0 LANGUAGES C CXX)

# Library
# add_subdirectory(matplotplusplus)

list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/third_party/matplotplusplus/install")
find_package(Matplot++ REQUIRED)
include(GNUInstallDirs)

# Our Project
set(SOURCE_FILES
    src/main.cpp
    src/RK4.cpp
    src/MatrixMath.cpp
    src/PlotData.cpp
)

include_directories(
    include/
)


add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC Matplot++::matplot)
