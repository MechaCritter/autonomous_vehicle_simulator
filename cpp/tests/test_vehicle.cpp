#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <spdlog/spdlog.h>
#include "../vehicle/Vehicle.h"
#include "../map/Map2D.h"
#include "../sensors/Lidar2D.h"

// Logger for test output
static std::shared_ptr<spdlog::logger> testLogger = spdlog::stdout_color_mt("VehicleTest");
// Use same debug directory as other tests
static std::filesystem::path output_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

// other global vars
static float step_size = 0.001;
// Parameter structure for vehicle kinematics tests
struct VehicleKinematicsParam {
    Pose2D init_pose;
    int vehicle_lenght; // meters
    int vehicle_width;  // meters
    double speed;
    double steering_angle; // radians
    double timespan; // seconds
    Pose2D expected_pose;
};

// Test fixture for parameterized Vehicle kinematics
class VehicleKinematicsTest : public ::testing::TestWithParam<VehicleKinematicsParam> { };

TEST_P(VehicleKinematicsTest, UpdatePose) {
    VehicleKinematicsParam P = GetParam();
    Vehicle vehicle(P.init_pose, P.vehicle_lenght, P.vehicle_width, {255,255,255}, 0.0);  // color white (just for instantiation)
    vehicle.setSpeedAndSteeringAngle(P.speed, P.steering_angle);

    // update the vehicle in a few time steps
    for (float t = 0; t < P.timespan; t += step_size) {
        vehicle.updateFor(step_size);
    }
    Pose2D result = vehicle.pose();
    testLogger->info("Updated pose: ({:.3f}, {:.3f}, {:.3f} rad)", result.x, result.y, result.theta);
    // Allow a tiny tolerance for floating-point arithmetic
    double tol = 2e-3;
    EXPECT_NEAR(result.x, P.expected_pose.x, tol);
    EXPECT_NEAR(result.y, P.expected_pose.y, tol);
    EXPECT_NEAR(result.theta, P.expected_pose.theta, tol);
}

// Instantiate test cases with different motion parameters
INSTANTIATE_TEST_SUITE_P(BasicMoves, VehicleKinematicsTest, ::testing::Values(
    // 1. Straight motion: 1 m/s for 2s -> expect 2m forward in x
    VehicleKinematicsParam{{0,0,0}, 4, 2, 1.0, 0.0, 2.0, {2.0, 0.0, 0.0}},
    // 2. Straight motion at 45°:  sqrt(2) m/s for 1s -> ~1 m in x and y (45 deg)
    VehicleKinematicsParam{{0,0,M_PI_4},  4, 2, std::sqrt(2.0), 0.0, 1.0, {1.0, 1.0, M_PI_4}},
    // 3. STeerign angle = 60°: 2 m/s for 2s
    //  According to the equation x_dot(t) = v * cos(theta_0 + yaw_rate * t)
    // => x(t) = x_0 + integral_0_t(v * cos(theta_0 + yaw_rate * t)) dt
    // = x_0 + v / yaw_rate * (sin(theta_0 + yaw_rate * t) - sin(theta_0))
    // = 12.27944
    // analogously, y = 12.68019
    // theta = sqrt(3) (since yaw_rate = v * tan(steering_angle) / length = sqrt(3) / 2.0)
    VehicleKinematicsParam{{10,10,0}, 4, 2, 2.0, M_PI/3, 2.0, {12.27944, 12.68019, std::sqrt(3)}},
    // 4. The same as above, but with a non-zero initial orientation
    // theta_1 = theta_0 + yaw_rate * t = PI/3 + sqrt(3) * 2.0 / 2.0 = sqrt(3) + M_PI/3
    VehicleKinematicsParam{{10,10,M_PI/3}, 4, 2, 2.0, M_PI/3, 2.0,
                           { 8.81861, 13.31415, std::sqrt(3) + M_PI/3 }}
));

#ifdef WITH_OPENCV_DEBUG
TEST(VehicleSimulationTest, VehiclePathVideo) {
    // Load default map
    Map2D map = Map2D::loadMap(DEFAULT_MAP_FILE, DEFAULT_MAP_RESOLUTION);
    // Initialize vehicle and sensor
    Pose2D start_v1{10.0, 10.0, M_PI/2};
    Pose2D start_v2{20.0, 10.0, M_PI/2};
    Lidar2D lidar("lidar", "roof", &map, 10, start_v1);
    Vehicle vehicle1(start_v1, 10.0, 5.0, {0, 0, 0}, 0.0);  // black car
    Vehicle vehicle2(start_v2, 10.0, 5.0, {0, 0, 0}, 0.0);  // black car
    vehicle1.addSensor(&lidar, Vehicle::MountSide::Front);
    map.addObject(vehicle1);
    map.addObject(vehicle2);

    // register vehicle on map
    // map.addVehicle(vehicle);

    map.startSimulationFor(10);
    // for (int i = 0; i < lidar.nBeams(); ++i) {
    //     double rel_angle = lidar.angleMin() + i * lidar.angleIncrement();
    //     std::vector<std::pair<int,int>> cells;
    //     double range = lidar.castRay(rel_angle, map, &cells);
    //     cv::Mat frame = map.renderToMat();
    //     // draw vehicle (rotated rectangle) and Lidar beams as shown above...
    //     // [For brevity, the drawing code is identical to the snippet provided earlier]
    //     // ...
    // }
    vehicle1.setSpeedAndSteeringAngle(4, M_PI/6);
    vehicle2.setSpeedAndSteeringAngle(7, 0.0); // straight line

    // for (float t = 0; t < 10; t += step_size) {
    //     testLogger->info("Vehicle at ({:.1f},{:.1f}), theta {:.2f}", vehicle.pose().x,
    //         vehicle.pose().y, vehicle.pose().theta);
    //     vehicle.update(step_size);
    //     // cv::Mat frame = map.renderToMat();
    //     map.update();
    // }
    // sleep while simulation is running
    while (map.simulationActive()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Optionally, you can log the vehicle's position at each step
        testLogger->info("Vehicle1 at ({:.1f},{:.1f}), theta {:.2f}",
                         vehicle1.pose().x, vehicle1.pose().y, vehicle1.pose().theta);
        testLogger->info("Lidar at ({:.1f},{:.1f}), theta {:.2f}",
                         lidar.pose().x, lidar.pose().y, lidar.pose().theta);
        testLogger->info("Vehicle2 at ({:.1f},{:.1f}), theta {:.2f}",
                         vehicle2.pose().x, vehicle2.pose().y, vehicle2.pose().theta);
    }
    map.endSimulation();
    std::filesystem::path videoFile = output_debug_path / "vehicle_path.mp4";
    map.flushFrames(videoFile.string());
    // Verify that video file was created and is not empty
    ASSERT_TRUE(std::filesystem::exists(videoFile));
    EXPECT_GT(std::filesystem::file_size(videoFile), 0);
    // (Optional) further checks: e.g., final vehicle position on map is correct
    Pose2D finalPose = vehicle1.pose();
    // EXPECT_NEAR(finalPose.x, lastWaypoint.x, 1e-6);
    // EXPECT_NEAR(finalPose.y, lastWaypoint.y, 1e-6);
    // EXPECT_NEAR(finalPose.theta, lastWaypoint.theta, 1e-6);
}
#endif

