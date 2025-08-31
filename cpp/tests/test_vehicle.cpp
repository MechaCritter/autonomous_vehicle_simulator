#include "gtest/gtest.h"
#include "map/Map2D.h"
#include "objects/Vehicle.h"
#include "objects/sensors/Lidar2D.h"
#include "objects/Road.h"
#include <../setup/Setup.h>
#include <cmath>

static std::filesystem::path output_debug_path_testVehicle = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";
/**
 * @test Vehicle stops on road after motor force is set to zero. This method checks
 * whether the friction implementation works.
 */
TEST(Vehicle, StopsOnRoadDueToFriction)
{
    setupWorld();
    Map2D map(500, 500);

    // Lay out a straight road band in the middle: y in [45, 55] px (≈ 4–6 m)
    for (int i = 0; i < 10; i++) {
        auto road = std::make_unique<Road>(10, 10, 10.0f * i, 5.0f);
        map.addObject(std::move(road));
    }

    auto car = std::make_unique<Vehicle>(
        4.0f,
         2.0f,
        std::array<uint8_t, 3>{50, 50, 220},
        0.0f,
        0.0f,
        3.0f,
        5.0f,
        0.0f
    );

    // Store a reference to the car before moving it into the map
    Vehicle* car_ptr = car.get();
    map.addObject(std::move(car));
    car_ptr->setSpeed(3.0f);
    car_ptr->start();

    map.startSimulation();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    car_ptr->setMotorForce(0.0f);         // force becomes null after ~2s

    // Monitor speed for 10 seconds, printing every 1 second
    auto start_time = std::chrono::steady_clock::now();
    auto target_duration = std::chrono::seconds(15);

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
    std::filesystem::path videoFile = output_debug_path_testVehicle / "vehicle_stop_on_friction.mp4";

    map.flushFrames(videoFile.string());
    // the vehicle should halt due to friction within 10s
    float speed = car_ptr->speed();

    EXPECT_NEAR(speed, 0.0f, 0.001f); // allow small tolerance
    destroyWorld();
}

/**
 * @brief in this test, two cars, each equipped with a LIDAR, should be able to detect each
 * other as they come close (<= 5m). After that distance, a brake force of -1000N is applied
 *
 * This test expects both vehicles to brake and eventually stop before colliding!
*/
TEST(Vehicle, StopWhenMeetingAnotherVehicle)
{
    setupWorld();
    Map2D map(800, 800);

    // Lay out a straight road band
    for (int i = 0; i < 15; i++) {
        auto road = std::make_unique<Road>(10, 10, 5.0f * i + 5.0f, 5.0f);
        map.addObject(std::move(road));
    }

    // two cars facing each other on the same y position
    auto car_left = std::make_unique<Vehicle>(
        4.0f,
         2.0f,
        std::array<uint8_t, 3>{255, 0, 255},
        0.0,
        .0f,
        3.0f,
        5.0f,
        0.0f
    );
    auto lidar_car_left = std::make_unique<Lidar2D>("lidar_left", "front", 60);
    lidar_car_left->setMap(&map);
    car_left->addSensor(std::move(lidar_car_left), Vehicle::MountSide::Front, 0.0f);

    auto car_right = std::make_unique<Vehicle>(
        4.0f,
        2.0f,
        std::array<uint8_t, 3>{255, 0, 255},
        M_PI,
        0.0,
        78.0f,
        5.0f,
        0.0f
    );

    auto lidar_car_right = std::make_unique<Lidar2D>("lidar_right", "front", 60);
    lidar_car_right->setMap(&map);
    car_right->addSensor(std::move(lidar_car_right), Vehicle::MountSide::Front, 0.0f);

    // Store a reference to the car before moving it into the map
    Vehicle* car_left_ptr = car_left.get();
    Vehicle* car_right_ptr = car_right.get();
    map.addObject(std::move(car_left));
    map.addObject(std::move(car_right));
    car_left_ptr->setSpeed(35.0f);
    car_right_ptr->setSpeed(35.0f);
    car_left_ptr->start();
    car_right_ptr->start();

    map.startSimulation();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Monitor speed for 10 seconds, printing every 1 second
    auto start_time = std::chrono::steady_clock::now();
    auto target_duration = std::chrono::seconds(20);

    while (std::chrono::steady_clock::now() - start_time < target_duration) {
        car_left_ptr->printPhysicalProperties();
        auto sensor_carleft_data = car_left_ptr->sensors()[0]->generateData();
        auto &scan_left = sensor_carleft_data->lidar_scan_2d();
        int idx_left = scan_left.ranges_size() / 2; // middle ray
        float dist_left = scan_left.ranges(idx_left);
        if (dist_left <= 15.0f) {
            std::cout << "[GOOD] Distance: " << dist_left << " m <= 5.0f m => braking!\n";
            car_left_ptr->setMotorForce(-10000.0f); // negative force => brake
        }

        car_right_ptr->printPhysicalProperties();
        auto sensor_carright_data = car_right_ptr->sensors()[0]->generateData();
        auto &scan_right = sensor_carright_data->lidar_scan_2d();
        int idx_right = scan_right.ranges_size() / 2; // middle ray
        float dist_right = scan_right.ranges(idx_right);
        if (dist_right <= 10.0f) {
            std::cout << "[GOOD] Distance: " << dist_right << " m <= 5.0f m => braking!\n";
            car_right_ptr->setMotorForce(-10000.0f); // negative force => brake
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "\n\n";
    }

    map.endSimulation();
    std::filesystem::path videoFile = output_debug_path_testVehicle / "vehicle_stop_avoid_collision.mp4";

    map.flushFrames(videoFile.string());
    destroyWorld();
}
