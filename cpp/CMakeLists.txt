cmake_minimum_required(VERSION 3.28)
project(Vehicle-Simulator)

set(CMAKE_CXX_STANDARD 23)

# Add local install dir to search path
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
include_directories("$ENV{HOME}/.local/include")
include_directories("$ENV{HOME}/.local/lib")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

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

# Path to the plugin
find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXECUTABLE)
    message(FATAL_ERROR "grpc_cpp_plugin not found")
else()
    message(STATUS "Found gRPC C++ plugin: ${GRPC_CPP_PLUGIN_EXECUTABLE}")
endif()

# subdirectories
add_subdirectory(grpc_service)

# Adding the generated protobuf sources to your target.
set(PROTO_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/proto/generated/sensor_data.pb.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/proto/generated/sensor_data.grpc.pb.cc)

# google test
## Add tests
enable_testing()
add_subdirectory(tests)

# Executable
add_executable(vehicle_simulator.exe
        main.cpp
        sensors/Sensor.cpp
        sensors/Lidar2D.cpp
        map/Map2D.cpp
        utils/utils.cpp
        ${PROTO_SRCS}
)

# Link libs to executable
target_link_libraries(vehicle_simulator.exe
        PRIVATE
            spdlog
            protobuf::libprotobuf
)
