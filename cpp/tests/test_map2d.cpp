#include <gtest/gtest.h>
#include "../map/Map2D.h"
#include "../objects/Vehicle.h"
#include "../objects/Obstacle.h"

// Logger for test output
static std::shared_ptr<spdlog::logger> testLogger = spdlog::stdout_color_mt("Map2DTest");
// Use same debug directory as other tests
static std::filesystem::path output_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

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

TEST(Map2D, VideoGeneration) {
    setupWorld();
    // Load default map
    Map2D map = Map2D::loadMap(DEFAULT_MAP_FILE, DEFAULT_MAP_RESOLUTION);
    Vehicle vehicle1(2.5, 1.5,
        {255, 0, 255},  // pink car
        M_PI/6,
        0.0,
        0.0,
        20.0,
        0.0);

    Vehicle vehicle2(2.5, 1.5,
        {255, 0, 0},
        M_PI/2,
        0.0,
        20.0,
        10.0,
        0);
    Obstacle obs1(2.0, 2.0, 20.0, 40.0, 0.0);
    map.addObject(vehicle1);
    map.addObject(vehicle2);
    map.addObject(obs1);
    vehicle1.setSteeringAngle(0.0); // slight turn
    vehicle1.setMotorForce(10000);
    vehicle2.setSteeringAngle(0);
    vehicle2.setMotorForce(10000);
    vehicle1.start();
    vehicle2.start();
    testLogger->info("Vehicle1 has motor force {:.2f} N", vehicle1.motorForce());
    testLogger->info("vehicle 1 has speed: {:.2f} m/s", vehicle1.speed());
    // vehicle2.setSpeedAndSteeringAngle(2, 0.0); // straight line
    testLogger->info("Vehicle2 has motor force {:.2f} N", vehicle2.motorForce());
    testLogger->info("vehicle 2 has speed: {:.2f} m/s", vehicle2.speed());

    map.startSimulation();
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    testLogger->info("Speed of vehicle 1 after 10s: {:.2f} m/s", vehicle1.speed());
    testLogger->info("Speed of vehicle 2 after 10s: {:.2f} m/s", vehicle2.speed());
    map.endSimulation();
    std::filesystem::path videoFile = output_debug_path / "vehicle_path.mp4";
    map.flushFrames(videoFile.string());
    // Verify that video file was created and is not empty
    ASSERT_TRUE(std::filesystem::exists(videoFile));
    EXPECT_GT(std::filesystem::file_size(videoFile), 0);
}
