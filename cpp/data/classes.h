#ifndef CLASSES_H
#define CLASSES_H
#include <variant>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <box2d/box2d.h>


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

/**
 * @brief Physics-related structures for Box2D integration
 */
namespace physics {
    inline std::unordered_map<Cell, float> friction_map = {
        {Cell::Free, 0.0f},      // Free space
        {Cell::Road, 0.9f},      // Road surface
        {Cell::Vehicle, 0.0f},   // Vehicle on vehicle
        {Cell::Lidar, 0.0f},     // Lidar sensor area
        {Cell::Obstacle, 0.9f},  // Obstacle area
        {Cell::Reserved, 0.00001f}   // Reserved area
    };

    /**
     * @brief Describes the shape properties of a physics object
     */
    enum class ShapeType {
        Box,
        Circle,
        Polygon
    };

}

#endif //CLASSES_H
