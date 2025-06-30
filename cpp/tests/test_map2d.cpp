#include <gtest/gtest.h>
#include "../map/Map2D.h"

// TODO: tests fail because of the relative path in the file "Map2D.cpp" (var DEFAULT_MAP_FILE)
// TODO: If a full path is provided instead, it works again. Find a solution to resolve the path issue!
TEST(Map2D, Access) {
    Map2D m(10,10,0.1, Cell::Free);
    m.setPx(3,3, Cell::Road);
    EXPECT_EQ(m.atPx(3,3), Cell::Road);
}

TEST(Map2D, Window) {
    Map2D g(20,20,0.1, Cell::Free);
    g.setPx(5,5, Cell::Obstacle);
    Map2D w = g.window(5,5,3);
    EXPECT_EQ(w.width(), 6);
    EXPECT_EQ(w.height(),6);
    EXPECT_EQ(w.atPx(3,3), Cell::Obstacle); // the central cell of the window should be the obstacle
}