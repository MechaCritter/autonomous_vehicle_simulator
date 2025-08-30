/**
 * @file test_vehicle.cpp
 * @brief Vehicle friction/stop behavior on road.
 *
 * This test creates a map, paints a horizontal "road" band, drops a vehicle onto it,
 * nudges it forward, sets motor force to zero after 2s, and then verifies the vehicle
 * stops (speed falls below a small threshold) due to terrain damping/friction logic.
 */

#include "gtest/gtest.h"
#include "map/Map2D.h"
#include "objects/Vehicle.h"
#include "objects/Road.h"
#include <../setup/Setup.h>
#include <cmath>

static std::filesystem::path output_debug_path_testVehicle = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";
/**
 * @test Vehicle stops on road after motor force is set to zero.
 */
TEST(Vehicle, StopsOnRoadDueToFriction)
{
    setupWorld();
    Map2D map(500, 500);

    // Lay out a straight road band in the middle: y in [45, 55] px (≈ 4–6 m)
    for (int i = 0; i < 10; i++) {
        auto road = std::make_unique<Road>(10, 10, 5.0f * i + 5.0f, 5.0f);
        map.addObject(std::move(road));
    }

    auto car = std::make_unique<Vehicle>(
        /*length*/ 4.0f,
         /*width*/ 2.0f,
        /*BGR*/ std::array<uint8_t, 3>{50, 50, 220},
        /*rotation*/ 0.0,
        /*init_speed*/ 0.0f,
        /*init_x*/ 3.0f,
        /*init_y*/ 5.0f,
        /*motor_force*/ 0.0f
    );

    // Store a reference to the car before moving it into the map
    Vehicle* car_ptr = car.get();
    map.addObject(std::move(car));
    car_ptr->setSpeed(10.0f);
    car_ptr->start();

    map.startSimulation();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    car_ptr->setMotorForce(0.0f);         // force becomes null after ~2s

    // Monitor speed for 10 seconds, printing every 1 second
    auto start_time = std::chrono::steady_clock::now();
    auto target_duration = std::chrono::seconds(10);

    while (std::chrono::steady_clock::now() - start_time < target_duration) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        float current_speed = car_ptr->speed();
        auto speed_vec = car_ptr->velocityVector();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time
        ).count();
        std::cout << "Time: " << elapsed << "s, Speed: " << current_speed << " m/s" << "\n";
        std::cout << "    Velocity Vector: (" << speed_vec.x << ", " << speed_vec.y << ")\n";
    }

    map.endSimulation();
    std::filesystem::path videoFile = output_debug_path_testVehicle / "vehicle_stop.mp4";

    map.flushFrames(videoFile.string());
    // the vehicle should halt due to friction within 10s
    float speed = car_ptr->speed();

    EXPECT_NEAR(speed, 0.0f, 0.001f); // allow small tolerance
    destroyWorld();
}
