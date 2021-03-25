cmake_minimum_required(VERSION 3.0)

#Configure project
project(claptrap)
set(PROJECT_DESCRIPTION "Ethercat master example for claptrap")

set(CMAKE_CXX_COMPILER "g++")

# Set C++ compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -fdiagnostics-color")

# Find ethercat library.
find_package(soem)

# Include source files from src directory.
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.c")
# Add directory containing the headers.
include_directories(include)

# Set executable target and link libraries.
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} soem)
install(TARGETS ${PROJECT_NAME} DESTINATION ".")
