cmake_minimum_required(VERSION 3.14.0)
project(NINT VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

# Library
add_subdirectory(matplotplusplus)

# Our Project
set(SOURCE_FILES
    src/main.cpp
    src/RK4.cpp
	src/MatrixMath.cpp
	)

set(HEADER_FILES
    src/RK4.hpp
	src/MatrixMath.hpp
	)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC matplot)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT NINT) # Automatically sets the startup project