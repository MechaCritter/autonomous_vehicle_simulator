#include <gtest/gtest.h>
#include "../map/Map2D.h"
#include "../objects/Vehicle.h"
#include "../objects/Obstacle.h"
#include <memory>
#include <vector>
#include <string>
#include <atomic>

// Logger for test output
static std::shared_ptr<spdlog::logger> testLogger = spdlog::stdout_color_mt("Map2DTest");
// Use same debug directory as other tests
static std::filesystem::path output_debug_path_testmap2d = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

// Global counter for unique video file names
static std::atomic<int> global_video_counter{1};

// Structure to define a vehicle configuration
struct VehicleConfig {
    float length, width;
    std::array<uint8_t,3> color_bgr;
    float rotation;
    float init_speed;
    float init_x, init_y;
    float motor_force;
    float steering_angle;

    VehicleConfig(float l, float w, std::array<uint8_t,3> color, float rot, float speed,
                  float x, float y, float force, float steer = 0.0f)
        : length(l), width(w), color_bgr(color), rotation(rot), init_speed(speed),
          init_x(x), init_y(y), motor_force(force), steering_angle(steer) {}
};

// Structure to define an obstacle configuration
struct ObstacleConfig {
    float length, width;
    float init_x, init_y;
    float rotation;

    ObstacleConfig(float l, float w, float x, float y, float rot = 0.0f)
        : length(l), width(w), init_x(x), init_y(y), rotation(rot) {}
};

// Test parameter structure
struct SimulationTestParams {
    std::string test_name;
    std::vector<VehicleConfig> vehicles;
    std::vector<ObstacleConfig> obstacles;
    int simulation_duration_seconds;
};

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

// Parametrized test class
class Map2DSimulationTest : public ::testing::TestWithParam<SimulationTestParams> {
protected:
    void SetUp() override {
        setupWorld();
    }

    void TearDown() override {
        destroyWorld();
    }
};

TEST_P(Map2DSimulationTest, SimulationScenarios) {
    auto params = GetParam();
    testLogger->info("Running simulation test: {}", params.test_name);

    // Load default map
    Map2D map = Map2D::loadMap(DEFAULT_MAP_FILE, DEFAULT_MAP_RESOLUTION);

    // Create and add vehicles
    std::vector<std::unique_ptr<Vehicle>> vehicles;
    for (size_t i = 0; i < params.vehicles.size(); ++i) {
        const auto& config = params.vehicles[i];
        auto vehicle = std::make_unique<Vehicle>(
            config.length, config.width, config.color_bgr, config.rotation,
            config.init_speed, config.init_x, config.init_y, config.motor_force
        );

        vehicle->setSteeringAngle(config.steering_angle);
        vehicle->setMotorForce(config.motor_force);
        vehicle->start();

        testLogger->info("Vehicle {} has motor force {:.2f} N and speed {:.2f} m/s",
                        i + 1, vehicle->motorForce(), vehicle->speed());

        map.addObject(*vehicle);
        vehicles.push_back(std::move(vehicle));
    }

    // Create and add obstacles
    std::vector<std::unique_ptr<Obstacle>> obstacles;
    for (size_t i = 0; i < params.obstacles.size(); ++i) {
        const auto& config = params.obstacles[i];
        auto obstacle = std::make_unique<Obstacle>(
            config.length, config.width, config.init_x, config.init_y, config.rotation
        );

        map.addObject(*obstacle);
        obstacles.push_back(std::move(obstacle));
    }

    // Start simulation
    map.startSimulation();
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(params.simulation_duration_seconds);

    while (std::chrono::steady_clock::now() - start_time < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    map.endSimulation();

    // Get current counter value and increment for next test
    int current_counter = global_video_counter.fetch_add(1);

    // Generate unique video filename using counter, test name
    std::string safe_test_name = params.test_name;
    std::replace(safe_test_name.begin(), safe_test_name.end(), ' ', '_');
    std::filesystem::path videoFile = output_debug_path_testmap2d /
        (std::to_string(current_counter) + "_" + safe_test_name + "_simulation.mp4");

    map.flushFrames(videoFile.string());

    // Verify that video file was created and is not empty
    ASSERT_TRUE(std::filesystem::exists(videoFile));
    EXPECT_GT(std::filesystem::file_size(videoFile), 0);
}

// Test parameters - define multiple scenarios
INSTANTIATE_TEST_SUITE_P(
    Map2DScenarios,
    Map2DSimulationTest,
    ::testing::Values(
        // Test 1: Original implementation - two vehicles with obstacle
        SimulationTestParams{
            "Original_Two_Vehicles_With_Obstacle",
            {
                VehicleConfig(2.5f, 1.5f, {255, 0, 255}, M_PI/6, 0.0f, 0.0f, 7.0f, 10000.0f, 0.0f),
                VehicleConfig(2.5f, 1.5f, {255, 0, 0}, 0, 0.0f, 0.0f, 15.0f, 10000.0f, 0.0f)
            },
            {
                ObstacleConfig(20.0f, 20.0f, 25.0f, 17.0f, M_PI/4)
            },
            15
        },

        // Test 2: Single vehicle obstacle course
        SimulationTestParams{
            "Single_Vehicle_Obstacle_Course",
            {
                VehicleConfig(3.0f, 1.8f, {0, 255, 0}, 0.0f, 0.0f, 10.0f, 10.0f, 8000.0f, M_PI/100)
            },
            {
                ObstacleConfig(15.0f, 15.0f, 30.0f, 40.0f, 0.0f),
                ObstacleConfig(7.0f, 7.0f, 50.0f, 13.0f, M_PI/4),
                ObstacleConfig(8.0f, 8.0f, 14.0f, 15.0f, 0.0f)
            },
            15
        },

        // Test 3: Multi-vehicle racing scenario
        SimulationTestParams{
            "Multi_Vehicle_Race",
            {
                VehicleConfig(2.0f, 1.2f, {255, 255, 0}, 0.0f, 10.0f, 10.0f, 20.0f, 12000.0f, 0.0f),
                VehicleConfig(2.2f, 1.4f, {0, 0, 255}, M_PI, 0.0f, 38.5f, 15.0f, 11000.0f, -M_PI/30),
                VehicleConfig(1.8f, 1.0f, {255, 165, 0}, -2 * M_PI/3, 0.0f, 40.5f, 40.0f, 13000.0f, M_PI/30),
                VehicleConfig(2.1f, 1.3f, {128, 0, 128}, 0.0f, 0.0f, 42.5f, 5.0f, 10500.0f, 0.0f)
            },
            {
                ObstacleConfig(30.0f, 5.0f, 20.0f, 20.0f, 0.0f)
            },
            15
        }
    )
);

