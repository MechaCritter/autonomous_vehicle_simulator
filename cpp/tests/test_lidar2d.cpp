#include <gtest/gtest.h>
#include <cmath>
#include <include/spdlog/spdlog.h>
#include <filesystem>
#include <include/spdlog/sinks/basic_file_sink.h>
#include "../objects/sensors/Lidar2D.h"
#include "../objects/Vehicle.h"
#include "../objects/Free.h"
#include "../objects/Obstacle.h"
#include "../objects/Road.h"
#include "../map/Map2D.h"
#include "../data/DataClasses.h"

// global var
std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("Testing");
std::filesystem::path output_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

#ifdef WITH_OPENCV_DEBUG                                       // build flag
constexpr bool DEBUG_IMG = true;
#else
constexpr bool DEBUG_IMG = false;
#endif

// TODO: use this RayDistanceResult for the test. Build a dictionary mapping each Ray Distance to the desired exp_distance.
// TODO: the exp_distance is called right after declaring the struct.
struct RayDistanceResult {
    float init_x;
    float init_y;
    float init_theta;
    float rel_angle;
    std::array<double, 2> obstacle_coord;
    float exp_distance;
};

struct LidarParam {
    int test_id; // just for logging
    float  res;          // [m/px]
    int    map_width; // map's width
    int    map_height; // map's height
    int    obs_x, obs_y; // obstacle pixel
    int    px,   py;     // lidar pixel
    float    theta_in_deg; // lidar orientation in degrees
    Cell   obstacle_cls; // what to write
};

class LidarParamTest : public ::testing::TestWithParam<LidarParam> {};


/**
* @brief Compute the expected distance between the lidar and the obstacle
* as well as the index of the first beam that should hit the obstacle. The beam
* is determined using simple trigonometry based on the lidar's pose and the
* obstacle's position.
*
* The distance is Euclidean unless the obstacle cell is `Free` or `Road`,
* in which case the beam should pass through and the expected distance is
* the sensor's maximum range.
*
* @param P               Current test parameters.
* @param scan_angle_min  The minimum angle of the lidar scan in radians.
* @param scan_ranges_size The number of ranges in the lidar scan.
* @param scan_angle_increment The angle increment between each beam in radians.
* @param scan_max_range  The maximum range of the lidar sensor in meters.
* @return std::pair<double, int> A pair containing the expected distance to the obstacle
*  and the index of the first beam that should hit the obstacle.
*/
static std::pair<double, int>
expected_distance_and_angle_diff(
    const LidarParam& P,
    float scan_angle_min,
    float scan_ranges_size,
    float scan_angle_increment,
    float scan_max_range
    )
{
    float theta_in_rad = P.theta_in_deg * M_PI / 180.0f; // convert degrees to radians
    const float dx = (P.obs_x - P.px) * P.res;
    const float dy = (P.obs_y - P.py) * P.res;
    const float angle = std::atan2(dy, dx);             // relative to x-axis
    const double ang_min =  scan_angle_min + theta_in_rad;
    const double ang_max =  scan_angle_min +
                            (scan_ranges_size-1)*scan_angle_increment + theta_in_rad; // -1 because the first beam is already angle_min
    const bool in_fov = angle >= ang_min && angle <= ang_max;
    double expected_distance;
    int expected_idx = 0; // if the lidar never hits, simply use index 0 (which should return the max range)

    // if the obstacle is a Free cell, the ray should go through
    // else: simply Pythagorean theorem
    if (P.obstacle_cls == Cell::Free || P.obstacle_cls == Cell::Road)
    {
        expected_distance = scan_max_range;
        logger->info("For test ID {}, obstacle is a Free or Road cell, so the expected distance is the maximum range of the lidar = {}.",
        P.test_id, scan_max_range);
    }
    else if (!in_fov)
    {
        expected_distance = scan_max_range;
        logger->info("For test ID {}, obstacle is not in FOV, so the expected distance is the maximum range of the lidar = {}.",
      P.test_id, scan_max_range);
    }
    else
    {
        expected_distance = std::min(std::sqrt(dx * dx + dy * dy), scan_max_range);
        expected_idx = static_cast<int>((angle - ang_min)/scan_angle_increment);
        logger->info("For test ID {}, obstacle is in FOV, so the expected distance is the Euclidean distance of {:.2f} m, "
    "but capped to the lidar's maximum range of {}. Expected index of the first beam hitting the obstacle is {}.",
      P.test_id, expected_distance, scan_max_range, expected_idx);
    }
    return {expected_distance, expected_idx};
}

TEST_P(LidarParamTest, GenerateData)
{
    const auto P = GetParam();
    logger->info("Running LidarParamTest with parameters: res={}, width={}, height={}, obs_x={}, obs_y={}, px={}, py={}, obstacle_cls={}. Test ID: {}",
                  P.res, P.map_width, P.map_height, P.obs_x, P.obs_y, P.px, P.py, static_cast<int>(P.obstacle_cls), P.test_id);
    logger->info("Output debug path: {}", output_debug_path.string());
    setupWorld();
    Map2D map(P.map_width, P.map_height);
    map.setResolution(P.res);
    // map.setPx(P.obs_x, P.obs_y, P.obstacle_cls);
    if (P.obstacle_cls == Cell::Vehicle) {
        auto vehicle = std::make_unique<Vehicle>(1.0f, 1.0f, std::array<uint8_t,3>{255,0,0}, 0.0f, 0.0f,
                                                 P.obs_x * P.res, P.obs_y * P.res, 0.0f);
        map.addObject(std::move(vehicle));
    }
    else if (P.obstacle_cls == Cell::Obstacle) {
        auto obstacle = std::make_unique<Obstacle>(1.0f, 1.0f,
                                                   P.obs_x * P.res, P.obs_y * P.res, 0.0f);
        map.addObject(std::move(obstacle));
    }
    else if (P.obstacle_cls == Cell::Road) {
        auto road = std::make_unique<Road>(1.0f, 1.0f,
                                           P.obs_x * P.res, P.obs_y * P.res, 0.0f);
        map.addObject(std::move(road));
    }
    else if (P.obstacle_cls == Cell::Free) {
        auto free = std::make_unique<Free>(1.0f, 1.0f,
                                           P.obs_x * P.res, P.obs_y * P.res, 0.0f);
        map.addObject(std::move(free));
    }

    // tolerance = half the resolution + a tolerance of 1e-3
    float tol = P.res * 0.5 + 1e-3;

    float theta_in_rad = P.theta_in_deg * M_PI / 180.0f; // convert degrees to radians
    std::string lidar_name = "lid_testid" + std::to_string(P.test_id);
    Lidar2D lidar(lidar_name, "front", 10, 2, 2, theta_in_rad, P.px * P.res,P.py * P.res);
    lidar.setMap(&map);
    std::unique_ptr<sensor_data::SensorData> data;

#ifdef WITH_OPENCV_DEBUG
    if constexpr (DEBUG_IMG) {
        // Generate debug video when DEBUG_IMG is enabled
        std::string video_filename = (output_debug_path /
            (lidar_name + "_debug.mp4")).string();
        data = lidar.generateDataWithDebugVideo(video_filename);
        logger->info("Debug video saved to: {}", video_filename);
    } else {
        // Normal data generation without debug visualization
        data = lidar.generateData();
    }
#else
    // Always use normal data generation when WITH_OPENCV_DEBUG is not defined
    data = lidar.generateData();
#endif

    ASSERT_TRUE(data->has_lidar_scan_2d());
    auto& scan = data->lidar_scan_2d();

    const auto [expected_distance, idx] = expected_distance_and_angle_diff(
        P, scan.angle_min(), scan.ranges_size(), scan.angle_increment(), scan.max_range());
    // the upper limit has to be the lidar's maximum range
    EXPECT_NEAR(scan.ranges(idx), expected_distance, tol); // +- resolution

    // line breaks for the next test
    std::cout << "---------------------------------------------------\n" << std::endl;
    destroyWorld();
}

INSTANTIATE_TEST_SUITE_P(
    VariedMaps,
    LidarParamTest,
    ::testing::Values(
        LidarParam{0, 0.1f, 60, 60, 40,50, 10,10, 0, Cell::Vehicle}, // hit
        LidarParam{1, 0.1f, 60, 60, 40,50, 10,10, 180, Cell::Vehicle}, // rotate 180 degrees => should not hit because will not be in FOV for sure
        LidarParam{2, 0.1f, 60, 60, 40,50, 10,10, 0, Cell::Free}, // ray should go through => yield max range
        LidarParam{3, 0.1f, 60, 60, 40,50, 10,10, 0, Cell::Road}, // ray should go through => yield max range
        LidarParam{4, 0.5f, 80, 40, 20,20, 0,20, 0, Cell::Obstacle}, // exactly at max_range
        LidarParam{5, 0.51f, 80, 40, 20,20, 0,20, 0, Cell::Obstacle}, // just a little bit above max_range => should yield max range
        LidarParam{6, 0.5f, 80, 20, 70,10,  60, 5,  0, Cell::Free},
        LidarParam{7, 0.5f, 30, 70,  6,40, 7,45, 0, Cell::Vehicle},   // outside FoV, but within max range

        // // TODO: the test below fails!! fail when too close to each other??
        LidarParam{8, 0.2f, 30, 70,  9,40, 7,41, 0, Cell::Vehicle}
        // // TODO: add test with different lidar poses!
    )
);

TEST(LidarBox2D, DoesNotHitOwnVehicle)
{
    setupWorld();
    Map2D map(200, 200);
    map.setResolution(0.1f); // 10 cm/px

    // Host vehicle at (5m, 5m), facing +X; mount lidar at Front (offset 0)
    Vehicle host(4.0f, 2.0f, {50,50,220}, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f);
    auto lidar = std::make_unique<Lidar2D>("lidar_front", "front", 20, 0.05f, 0.05f, 0.0f, 0.0f, 0.0f);
    Lidar2D* lidar_ptr = lidar.get(); // keep a raw ptr for later use
    host.addSensor(std::move(lidar), Vehicle::MountSide::Front, 0.0f);
    host.setMap(&map);

    // Cast straight ahead: must NOT hit the host vehicle
    const double d = lidar_ptr->castRay(0.0, map);
    EXPECT_DOUBLE_EQ(d, lidar_ptr->maxRange()); // requirement (1)
    destroyWorld();
}

TEST(LidarBox2D, HitsOtherVehicleAtKnownDistance)
{
    setupWorld();
    Map2D map(400, 200);
    map.setResolution(0.1f);

    Vehicle host(4.0f, 2.0f, {50,50,220}, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f);
    auto lidar = std::make_unique<Lidar2D>("lidar_front", "front", 20, 0.05f, 0.05f, 0.0f, 0.0f, 0.0f);
    Lidar2D* lidar_ptr = lidar.get(); // keep a raw ptr for later use
    host.addSensor(std::move(lidar), Vehicle::MountSide::Front, 0.0f);
    host.setMap(&map);

    // Place target vehicle 3.0m ahead of the host's front face, same Y
    const float targetLen = 4.0f;
    const float targetHalf = 0.5f * targetLen;
    // Host front face is at host_x + 0.5*host_len = 5 + 2 = 7
    // Put target center so its near face is at 7 + 3.0 => center = near + halfLen
    Vehicle target(targetLen, 2.0f, {220,50,50}, 0.0f, 0.0f, /*center x*/ 7.0f + 3.0f + targetHalf, 5.0f, 0.0f);
    target.setMap(&map);

    const double d = lidar_ptr->castRay(0.0, map);
    EXPECT_NEAR(d, 3.0, 1e-3); // within 1mm
    destroyWorld();
}