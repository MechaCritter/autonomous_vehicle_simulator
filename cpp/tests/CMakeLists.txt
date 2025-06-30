#cmake_minimum_required(VERSION 3.28)
#project(Vehicle-Simulator-Testing)
#set(CMAKE_CXX_STANDARD 23)
#
## Create a small library with the production sources you need in tests
#add_library(testlib
#        ${PROJECT_SOURCE_DIR}/map/Map2D.cpp
#        ${PROJECT_SOURCE_DIR}/utils/utils.cpp
#        ${PROJECT_SOURCE_DIR}/sensors/Lidar2D.cpp
#        ${PROJECT_SOURCE_DIR}/sensors/Sensor.cpp
#        ${PROTO_SRCS}
#)
#target_include_directories(testlib PUBLIC
#        ${PROJECT_SOURCE_DIR}   # so tests can do #include "map/Map2D.h"
#)
## Add spdlog to the library (otherwise, error will be thrown that "spdlog/spdlog.h cannot be found")
#target_link_libraries(testlib PUBLIC
#        spdlog
#        ${OpenCV_LIBS}
#)
## ----- Test executable --------------------------------------------------
#add_executable(code_test
#        test_map2d.cpp
#        test_lidar2d.cpp
#)
#
## Add opencv debug flag
#if (WITH_OPENCV_DEBUG)
#    message(STATUS "OpenCV debug mode enabled. Video outputs for sensors will be generated")
#    target_compile_definitions(code_test
#            PRIVATE WITH_OPENCV_DEBUG)
#endif ()
#
#target_link_libraries(code_test
#        PRIVATE
#        testlib
#        protobuf::libprotobuf
#        spdlog
#        ${OpenCV_LIBS}
#        gtest_main         # gives you int main(int, char**)
#)
#
## Let CTest know about it
#include(GoogleTest)          # comes with googletest
#gtest_discover_tests(code_test)
#
## Add debugging information
#if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#    target_compile_definitions(code_test PRIVATE LIDAR_DEBUG)
#endif ()