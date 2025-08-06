#ifndef CLASSES_H
#define CLASSES_H
#include <variant>
#include <cstdint>
#include <unordered_map>


/**
 * @brief Represents a 2D pose with position and orientation
 * @details This structure defines a pose in 2D space consisting of:
 *          - x: position along x-axis in meters
 *          - y: position along y-axis in meters
 *          - theta: orientation angle in radians
 */
struct Pose2D {
    double x;
    double y;
    double theta;
};

// Forward declarations for sensor data types
struct LidarScan2D;
struct Odometry2D;
struct Imu2D;
struct CameraFrame2D;

// Sensor data types
using SensorDataVariant = std::variant<
    LidarScan2D,
    Odometry2D,
    Imu2D,
    CameraFrame2D>;

/**
 * @brief Represents different types of cells in a grid-based environment.
 * This will be used for the `Map2D` class to represent the state of each cell in the map.
 */
enum class Cell : std::uint8_t {
    Unknown = 0,
    Free = 1,
    Road = 2,
    Vehicle = 3,
    Lidar = 4,
    Obstacle = 5,
    Reserved = 6
};

#endif //CLASSES_H
