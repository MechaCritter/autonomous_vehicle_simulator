#include <gtest/gtest.h>
#include <cmath>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <spdlog/sinks/basic_file_sink.h>
#include "../sensors/Lidar2D.h"
#include "../map/Map2D.h"
#include "../data/classes.h"

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
    int test_id; // just for logging
    float  res;          // [m/px]
    int    width;
    int    height;
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
* the sensor’s maximum range.
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
expected_distance_and_angle_diff(const LidarParam& P,
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
                  P.res, P.width, P.height, P.obs_x, P.obs_y, P.px, P.py, static_cast<int>(P.obstacle_cls), P.test_id);
    logger->info("Output debug path: {}", output_debug_path.string());

    Map2D map(P.width, P.height, P.res, Cell::Free);
    map.setPx(P.obs_x, P.obs_y, P.obstacle_cls);

    // tolerance = half the resolution + a tolerance of 1e-3
    float tol = P.res * 0.5 + 1e-3;

    float theta_in_rad = P.theta_in_deg * M_PI / 180.0f; // convert degrees to radians
    std::string lidar_name = "lid_testid" + std::to_string(P.test_id);
    Lidar2D lidar(lidar_name, "front",&map, 10, Pose2D{P.px * P.res,P.py * P.res, theta_in_rad});
    auto data = lidar.generateData();

    ASSERT_TRUE(data->has_lidar_scan_2d());
    auto& scan = data->lidar_scan_2d();

    const auto [expected_distance, idx] = expected_distance_and_angle_diff(
        P, scan.angle_min(), scan.ranges_size(), scan.angle_increment(), scan.max_range());
    // the upper limit has to be the lidar's maximum range
    EXPECT_NEAR(scan.ranges(idx), expected_distance, tol); // +- resolution

    // line breaks for the next test
    std::cout << "---------------------------------------------------\n" << std::endl;
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
        LidarParam{6, 0.5f, 80, 20, 70,10,  60, 5,  0, Cell::Reserved},
        LidarParam{7, 0.5f, 30, 70,  6,40, 7,45, 0, Cell::Vehicle},   // outside FoV, but within max range

        // // TODO: the test below fails!! fail when too close to each other??
        LidarParam{8, 0.2f, 30, 70,  9,40, 7,41, 0, Cell::Vehicle}
        // // TODO: add test with different lidar poses!
    )
);