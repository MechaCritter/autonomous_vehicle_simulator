#include <gtest/gtest.h>
#include <cmath>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <spdlog/sinks/basic_file_sink.h>
#include "../sensors/Lidar2D.h"
#include "../map/Map2D.h"
#include "../utils/utils.h"

// global var
std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("Testing");
std::filesystem::path output_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

#ifdef LIDAR_DEBUG                                       // build flag
constexpr bool DEBUG_IMG = true;
#else
constexpr bool DEBUG_IMG = false;
#endif

// TODO: use this RayDistanceResult for the test. Build a dictionary mapping each Ray Distance to the desired exp_distance.
// TODO: the exp_distance is called right after declaring the struct.
struct RayDistanceResult {
    Pose2D pose;
    double rel_angle;
    std::array<double, 2> obstacle_coord;
    double exp_distance;
};

struct LidarParam {
    float  res;          // [m/px]
    int    width;
    int    height;
    int    obs_x, obs_y; // obstacle pixel
    int    px,   py;     // lidar pixel
    float    theta_in_deg; // lidar orientation in degrees
    Cell   obstacle_cls; // what to write
};

class LidarParamTest : public ::testing::TestWithParam<LidarParam> {};

TEST_P(LidarParamTest, GenerateData)
{
    const auto P = GetParam();
    logger->info("Running LidarParamTest with parameters: res={}, width={}, height={}, obs_x={}, obs_y={}, px={}, py={}, obstacle_cls={}",
                  P.res, P.width, P.height, P.obs_x, P.obs_y, P.px, P.py, static_cast<int>(P.obstacle_cls));
    logger->info("Output debug path: {}", output_debug_path.string());

    Map2D map(P.width, P.height, P.res, Cell::Free);
    map.setPx(P.obs_x, P.obs_y, P.obstacle_cls);

    // tolerance = half the resolution + a tolerance of 1e-3
    float tol = P.res * 0.5 + 1e-3;

    float theta_in_rad = P.theta_in_deg * M_PI / 180.0f; // convert degrees to radians
    Lidar2D lidar("lid","front",&map, 10, Pose2D{P.px * P.res,P.py * P.res, theta_in_rad});
    auto data = lidar.generateData();

    ASSERT_TRUE(data->has_lidar_scan_2d());
    auto& scan = data->lidar_scan_2d();

    const float dx = (P.obs_x - P.px) * P.res;
    const float dy = (P.obs_y - P.py) * P.res;
    const float angle = std::atan2(dy, dx);             // relative to x-axis
    const double ang_min =  scan.angle_min() + theta_in_rad;
    const double ang_max =  scan.angle_min() +
                            (scan.ranges_size()-1)*scan.angle_increment() + theta_in_rad; // -1 because the first beam is already angle_min

    // if the obstacle is in the field of view, the distance should be equal to the expected distance.
    // Otherwise, it should be the maximum range of the lidar.
    const bool in_fov = angle >= ang_min && angle <= ang_max;
    double expected_distance;

    // if the obstacle is a Free cell, the ray should go through
    if (P.obstacle_cls == Cell::Free || P.obstacle_cls == Cell::Road) {
        logger->info("Obstacle is Free, so the ray should go through.");
        expected_distance = scan.max_range(); // ray should go through
    }
    // else: simply Pythagorean theorem
    else {expected_distance = std::sqrt(dx*dx + dy*dy);}

    // index of the beam
    int idx = static_cast<int>((angle - ang_min)/scan.angle_increment());

    // the upper limit has to be the lidar's maximum range
    if (expected_distance <= scan.max_range() && in_fov) {
        logger->info("Expected distance {} is within the lidar's max range {} and in FOV.",
              expected_distance, scan.max_range());
        EXPECT_NEAR(scan.ranges(idx), expected_distance, tol); // +- resolution
    }
    // if not in FOV but the distance is within the lidar's max range, the index should exceed the number of beams
    else if (expected_distance <= scan.max_range() && !in_fov)
    {
        logger->info("Expected distance {} is within the lidar's max range {}, but not in FOV.",
              expected_distance, scan.max_range());
        const int idx_diff = (angle < ang_min) ? -idx : idx - scan.ranges_size();
        const double angle_diff = (angle < ang_min) ? (ang_min - angle) : (angle - ang_max);
        EXPECT_NEAR(idx_diff*scan.angle_increment(), angle_diff, tol); // +- resolution
    }

    else
        {
        logger->info("Expected distance {} is greater than max range {}.",
              expected_distance, scan.max_range());
        EXPECT_NEAR(scan.max_range(), lidar.maxRange(), tol); // +- resolution
    }
}

INSTANTIATE_TEST_SUITE_P(
    VariedMaps,
    LidarParamTest,
    ::testing::Values(
        LidarParam{0.1f, 60, 60, 40,50, 10,10, 0, Cell::Vehicle}, // hit
        LidarParam{0.1f, 60, 60, 40,50, 10,10, 180, Cell::Vehicle}, // rotate 180 degrees => should not hit because will not be in FOV for sure
        LidarParam{0.1f, 60, 60, 40,50, 10,10, 0, Cell::Free}, // ray should go through => yield max range
        LidarParam{0.1f, 60, 60, 40,50, 10,10, 0, Cell::Road}, // ray should go through => yield max range
        LidarParam{0.5f, 80, 40, 20,20, 0,20, 0, Cell::Obstacle}, // exactly at max_range
        LidarParam{0.51f, 80, 40, 20,20, 0,20, 0, Cell::Obstacle}, // just a little bit above max_range => should yield max range
        LidarParam{0.5f, 80, 20, 70,10,  60, 5,  0, Cell::Reserved},
        LidarParam{0.5f, 30, 70,  6,40, 7,45, 0, Cell::Vehicle},   // outside FoV, but within max range

        // // TODO: the test below fails!! fail when too close to each other??
        LidarParam{0.2f, 30, 70,  9,40, 7,41, 0, Cell::Vehicle}
        // // TODO: add test with different lidar poses!
    )
);