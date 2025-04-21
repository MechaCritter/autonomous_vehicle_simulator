# Create a small library with the production sources you need in tests
add_library(map2d_testlib
        ${PROJECT_SOURCE_DIR}/map/Map2D.cpp
        ${PROJECT_SOURCE_DIR}/utils/utils.cpp
)
target_include_directories(map2d_testlib PUBLIC
        ${PROJECT_SOURCE_DIR}   # so tests can do #include "map/Map2D.h"
)

# ----- Test executable --------------------------------------------------
add_executable(test_map2d
        test_map2d.cpp
)

target_link_libraries(test_map2d
        PRIVATE
        map2d_testlib
        gtest_main         # gives you int main(int, char**)
)

# Let CTest know about it
include(GoogleTest)          # comes with googletest
gtest_discover_tests(test_map2d)
