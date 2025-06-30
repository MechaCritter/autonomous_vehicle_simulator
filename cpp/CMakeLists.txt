cmake_minimum_required(VERSION 3.28)
project(Vehicle-Simulator)
set(CMAKE_CXX_STANDARD 23)

# Add local install dir to search path
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
include_directories("$ENV{HOME}/.local/include")
include_directories("$ENV{HOME}/.local/lib")
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# -- add all relevant compile flags here! --- #
if (WITH_OPENCV_DEBUG)
    add_compile_definitions(WITH_OPENCV_DEBUG)
    message(STATUS "OpenCV debug mode enabled globally. Sensor's generateData method will generate video outputs.")
endif()

# Find necessary packages
## A. spdlog
include(FetchContent)
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.15.2
)
FetchContent_MakeAvailable(spdlog)
## B. google test
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.16.0            # latest stable at the moment
)
# Prevent GoogleTest from overriding our CMake settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(OpenSSL REQUIRED) # open-ssl is needed for gRPC
find_package(OpenCV REQUIRED) # opencv for video outputs when debugging sensors

# Path to the plugin
find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXECUTABLE)
    message(FATAL_ERROR "grpc_cpp_plugin not found")
else()
    message(STATUS "Found gRPC C++ plugin: ${GRPC_CPP_PLUGIN_EXECUTABLE}")
endif()


# Adding the generated protobuf sources to your target.
set(PROTO_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/proto/generated/sensor_data.pb.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/proto/generated/sensor_data.grpc.pb.cc)

# Executable
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
add_executable(vehicle_simulator.exe
        main.cpp
        sensors/Sensor.cpp
        sensors/Lidar2D.cpp
        map/Map2D.cpp
        utils/utils.cpp
        ${PROTO_SRCS}
)

# Add flags to build process to catch memory leaks. Only enable sanitizer in Debug builds
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(vehicle_simulator.exe PRIVATE -fsanitize=address -g)
    target_link_options(vehicle_simulator.exe PRIVATE -fsanitize=address)
endif()

# Link libs to executable
target_link_libraries(vehicle_simulator.exe
        PRIVATE
            spdlog
            protobuf::libprotobuf
            ${OpenCV_LIBS}
)

## custom flags for debugging
#if (WITH_OPENCV_DEBUG)
#    message(STATUS "OpenCV debug mode enabled.")
#    target_compile_definitions(vehicle_simulator.exe PRIVATE WITH_OPENCV_DEBUG)
#endif ()

# google test
## Add tests
enable_testing()

# Create a small library with the production sources you need in tests
add_library(testlib
        ${PROJECT_SOURCE_DIR}/map/Map2D.cpp
        ${PROJECT_SOURCE_DIR}/utils/utils.cpp
        ${PROJECT_SOURCE_DIR}/sensors/Lidar2D.cpp
        ${PROJECT_SOURCE_DIR}/sensors/Sensor.cpp
        ${PROTO_SRCS}
)
target_include_directories(testlib PUBLIC
        ${PROJECT_SOURCE_DIR}   # so tests can do #include "map/Map2D.h"
)
# Add spdlog to the library (otherwise, error will be thrown that "spdlog/spdlog.h cannot be found")
target_link_libraries(testlib PUBLIC
        spdlog
        ${OpenCV_LIBS}
)
# ----- Test executable --------------------------------------------------
add_executable(code_test
        ${PROJECT_SOURCE_DIR}/tests/test_map2d.cpp
        ${PROJECT_SOURCE_DIR}/tests/test_lidar2d.cpp
)

## Add opencv debug flag
#if (WITH_OPENCV_DEBUG)
#    message(STATUS "OpenCV debug mode enabled. Video outputs for sensors will be generated")
#    target_compile_definitions(code_test
#            PRIVATE WITH_OPENCV_DEBUG)
#endif ()

target_link_libraries(code_test
        PRIVATE
        testlib
        protobuf::libprotobuf
        spdlog
        ${OpenCV_LIBS}
        gtest_main         # gives you int main(int, char**)
)

# Let CTest know about it
include(GoogleTest)          # comes with googletest
gtest_discover_tests(code_test)

# Add debugging information
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(code_test PRIVATE LIDAR_DEBUG)
endif ()

### Create a small library with the production sources you need in tests
#add_library(testlib
#        ${PROJECT_SOURCE_DIR}/map/Map2D.cpp
#        ${PROJECT_SOURCE_DIR}/utils/utils.cpp
#        ${PROJECT_SOURCE_DIR}/sensors/Lidar2D.cpp
#)
#
#target_include_directories(testlib PUBLIC
#        ${PROJECT_SOURCE_DIR}   # so tests can do #include "map/Map2D.h"
#)
#
### ----- Test executable --------------------------------------------------
#add_executable(code_test
#        ${PROJECT_SOURCE_DIR}/tests/test_map2d.cpp
#        ${PROJECT_SOURCE_DIR}/tests/test_lidar2d.cpp
#)
#message(STATUS "Code test built with ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
#target_link_libraries(code_test
#        PRIVATE
#        testlib
#        gtest_main         # gives you int main(int, char**)
#)#
#
### Let CTest know about it
#include(GoogleTest)          # comes with googletest
#gtest_discover_tests(code_test)

