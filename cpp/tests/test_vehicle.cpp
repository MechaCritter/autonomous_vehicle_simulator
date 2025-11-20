#include "gtest/gtest.h"
#include "map/Map2D.h"
#include "../objects/vehicle/Vehicle.h"
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
    car_ptr->startUpdating();

    map.startSimulation();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    car_ptr->setMotorForce(0.0f);         // force becomes null after ~2s

    // Monitor speed for 10 seconds, printing every 1 second
    auto start_time = std::chrono::steady_clock::now();
    auto target_duration = std::chrono::seconds(15);

    while (std::chrono::steady_clock::now() - start_time < target_duration) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        float current_speed = car_ptr->absoluteSpeed();
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
    float speed = car_ptr->absoluteSpeed();

    EXPECT_NEAR(speed, 0.0f, 0.001f); // allow small tolerance
    destroyWorld();
}

/**
 * @brief in this test, two cars, each equipped with a LIDAR, should be able to detect each
 * other as they come close (<= 20m). After that distance, a strong braking force is applied.
 *
 * This test expects both vehicles to brake and eventually stop before colliding!
*/
TEST(Vehicle, StopWhenMeetingAnotherVehicle)
{
    setupWorld();
    Map2D map(800, 800);
    float lidar_max_range = 50.0f; // meters
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
        20.0f,
        3.0f,
        5.0f,
        0.0f
    );
    std::vector path_left = {
        b2Vec2{3.0f, 5.0f},   // start
        b2Vec2{95.0f, 5.0f}   // end
    };
    car_left->setGlobalPath(path_left);
    auto lidar_car_left = std::make_unique<Lidar2D>("lidar_left", "front", 60);
    lidar_car_left->setMaxRange(lidar_max_range);
    lidar_car_left->setMap(&map);
    car_left->addSensor(std::move(lidar_car_left), Vehicle::MountSide::Front, 0.0f);

    auto car_right = std::make_unique<Vehicle>(
        4.0f,
        2.0f,
        std::array<uint8_t, 3>{255, 0, 255},
        M_PI,
        20.0,
        78.0f,
        5.0f,
        0.0f
    );
    std::vector path_right = {
        b2Vec2{78.0f, 5.0f},   // start
        b2Vec2{3.0f, 5.0f}   // end
    };
    car_right->setGlobalPath(path_right);
    auto lidar_car_right = std::make_unique<Lidar2D>("lidar_right", "front", 60);
    lidar_car_right->setMaxRange(lidar_max_range);
    lidar_car_right->setMap(&map);
    car_right->addSensor(std::move(lidar_car_right), Vehicle::MountSide::Front, 0.0f);

    // Store a reference to the car before moving it into the map
    Vehicle* car_left_ptr = car_left.get();
    Vehicle* car_right_ptr = car_right.get();
    map.addObject(std::move(car_left));
    map.addObject(std::move(car_right));
    car_left_ptr->startUpdating();
    car_right_ptr->startUpdating();

    map.startSimulation();

    car_left_ptr->setDesiredAbsoulteSpeed(40.0f);
    car_right_ptr->setDesiredAbsoulteSpeed(40.0f);
    // Monitor speed, printing every 0.1 second
    auto start_time = std::chrono::steady_clock::now();
    auto target_duration = std::chrono::seconds(7);

    while (std::chrono::steady_clock::now() - start_time < target_duration) {
        // car_left_ptr->printPhysicalProperties();
        auto sensor_carleft_data = car_left_ptr->sensors()[0]->generateData();
        auto &scan_left = sensor_carleft_data->lidar_scan_2d();
        int idx_left = scan_left.ranges_size() / 2; // middle ray
        float dist_left = scan_left.ranges(idx_left);
        if (dist_left <= std::clamp(25.0f, 0.0f, lidar_max_range)) {
            std::cout << "[GOOD] Distance: " << dist_left << " m <= 5.0f m => braking!\n";
            car_left_ptr->brake(-20000); // negative force => brake
        }

        // car_right_ptr->printPhysicalProperties();
        auto sensor_carright_data = car_right_ptr->sensors()[0]->generateData();
        auto &scan_right = sensor_carright_data->lidar_scan_2d();
        int idx_right = scan_right.ranges_size() / 2; // middle ray
        float dist_right = scan_right.ranges(idx_right);
        if (dist_right <= std::clamp(25.0f, 0.0f, lidar_max_range)) {
            std::cout << "[GOOD] Distance: " << dist_right << " m <= 5.0f m => braking!\n";
            car_right_ptr->brake(-20000); // negative force => brake
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "\n\n";
    }

    map.endSimulation();
    std::filesystem::path videoFile = output_debug_path_testVehicle / "vehicle_stop_avoid_collision.mp4";

    map.flushFrames(videoFile.string());
    // vehicle moves on the straight line and should stop before colliding
    float speed_left = car_left_ptr->absoluteSpeed();
    auto [xleft, yleft] = car_left_ptr->position();
    float speed_right = car_right_ptr->absoluteSpeed();
    auto [xright, yright] = car_right_ptr->position();
    ASSERT_EQ(speed_left, 0.0f);
    ASSERT_EQ(speed_right, 0.0f);
    ASSERT_EQ(yleft, 5.0f);
    ASSERT_EQ(yright, 5.0f);
    destroyWorld();
}

/**
 * @brief This test checks whether the vehicle's PID controller can correct its path
 * after being disturbed by an external impulse. The vehicle is expected to return to
 * its planned route on the road after being pushed off course.
 */
TEST(Vehicle, PIDControllerPathCorrection) {
    setupWorld();
    Map2D map(800, 800);
    // Create a straight horizontal road composed of multiple segments
    for (int i = 0; i < 10; ++i) {
        auto road = std::make_unique<Road>(10.0f, 10.0f, 10.0f * i + 5.0, 5.0f);
        map.addObject(std::move(road));
    }
    // Place a vehicle on the road
    auto car = std::make_unique<Vehicle>(
        4.0f, 2.0f,
        std::array<uint8_t,3>{50, 50, 220},
        0.0f,
        0.0f,
        3.0f, 5.0f,
        12000.0f
    );
    Vehicle* car_ptr = car.get();
    map.addObject(std::move(car));
    // Plan route straight along the road
    std::vector path = {
        b2Vec2{3.0f, 5.0f},   // start
        b2Vec2{95.0f, 5.0f}   // end
    };
    car_ptr->setGlobalPath(path);
    car_ptr->startUpdating();
    map.startSimulation();

    // Let the vehicle drive for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // Apply a sudden lateral impulse to push the vehicle off its path
    b2Body_ApplyLinearImpulse(car_ptr->bodyId(),
                              b2Vec2{0.0f, 2500.0f},
                              car_ptr->position(),
                              true);
    // Wait a few more seconds for the PID to correct course
    auto timeout = std::chrono::seconds(60);
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // sets motor force to 0 when vehicle is almost at the goal
        if (95 - car_ptr->position().x < 3.0f) {
            car_ptr->brake();
        }
    }
    map.endSimulation();
    map.flushFrames((output_debug_path_testVehicle / "vehicle_pid_path_correction.mp4").string());
    // After disturbance, vehicle should realign close to the road center (y ≈ 5.0)
    float final_y = car_ptr->position().y;
    EXPECT_NEAR(final_y, 5.0f, 0.05f);
    destroyWorld();
}

/**
 * @brief This is a more challenging version of the PIDControllerPathCorrection test. Here, the vehicle
 * will move in a rectangular path, making 90-degree turns at each corner, while being disturbed.
 */
TEST(Vehicle, RectangularPath) {
    setupWorld();
    Map2D map(800, 800);
    // Create a straight horizontal road composed of multiple segments
    for (int i = 0; i < 10; ++i) {
        auto road_hor_upper = std::make_unique<Road>(10.0f, 10.0f, 10.0f * i + 5.0, 5.0f);
        auto road_hor_lower = std::make_unique<Road>(10.0f, 10.0f, 10.0f * i + 5.0f, 75.0f);
        auto road_ver_left = std::make_unique<Road>(10.0f, 10.0f, 5.0f, 10.0f * i + 5.0f);
        auto road_ver_right = std::make_unique<Road>(10.0f, 10.0f, 75.0f, 10.0f * i + 5.0f);
        map.addObject(std::move(road_ver_left));
        map.addObject(std::move(road_ver_right));
        map.addObject(std::move(road_hor_upper));
        map.addObject(std::move(road_hor_lower));
    }
    // Place a vehicle on the road
    auto car = std::make_unique<Vehicle>(
        6.0f, 3.0f,
        std::array<uint8_t,3>{50, 50, 220},
        0.0f,
        0.0f,
        3.0f, 5.0f,
        12000.0f
    );
    Vehicle* car_ptr = car.get();
    map.addObject(std::move(car));
    // Plan route straight along the road
    std::vector path = {
        b2Vec2{5.0f, 5.0f},
        b2Vec2{40.0f, 5.0f},
        b2Vec2{75.0f, 5.0f}  ,
        b2Vec2{75.0f, 40.0f} ,
        b2Vec2{75.0f, 75.0f},
        b2Vec2{40.0f, 75.0f} ,
        b2Vec2{5.0f, 75.0f}  ,
        b2Vec2{5.0f, 40.0f}  ,
        b2Vec2{5.0f, 5.0f}   ,
    };
    car_ptr->setGlobalPath(path);
    car_ptr->startUpdating();
    map.startSimulation();

    // Let the vehicle drive for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // Apply a sudden lateral impulse to push the vehicle off its path
    b2Body_ApplyLinearImpulse(car_ptr->bodyId(),
                              b2Vec2{0.0f, 2500.0f},
                              car_ptr->position(),
                              true);
    // Wait a few more seconds for the PID to correct course
    auto timeout = std::chrono::seconds(180);
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // // sets motor force to 0 when vehicle is almost at the goal
        // if (5 - car_ptr->position().x < 0.1f && 5 - car_ptr->position().y < 0.1f) {
        //     car_ptr->brake();
        // }
    }
    map.endSimulation();
    map.flushFrames((output_debug_path_testVehicle / "vehicle_pid_rectangular_path.mp4").string());
    // After disturbance, vehicle should realign close to the road center (y ≈ 5.0)
    float dist = b2Distance(car_ptr->position(), b2Vec2{5.0f, 5.0f});
    EXPECT_NEAR(dist, 0.0f, 0.5 * car_ptr->length()); // allows tolerance up to half the vehicle length
    destroyWorld();
}


