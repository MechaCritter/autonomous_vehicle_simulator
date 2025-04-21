//
// Created by vunha on 4/19/2025.
//

#include "Lidar2D.h"
#include <chrono>

std::unique_ptr<sensor_data::SensorData> Lidar2D::generateData(const std::vector<int>& input)
{
    auto data = std::make_unique<sensor_data::SensorData>();

    // Fill in header information
    auto header = data->mutable_header();
    // Here we use a dummy sensor_id, you might expose a getter from Sensor class if needed.
    header->set_sensor_id(0);
    header->set_name(name());
    header->set_sensor_type(protoType());

    // Use a simple timestamp (milliseconds since epoch)
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    header->set_timestamp(static_cast<uint64_t>(ms));

    // Fill in the LidarScan2D message
    auto lidar_scan = data->mutable_lidar_scan_2d();
    constexpr float angle_min = -1.57f;        // -90 degrees in radians.
    constexpr float angle_increment = 0.1f;      // Angle increment per ray.
    constexpr float max_range = 50.0f;           // Maximum range in meters.

    lidar_scan->set_angle_min(angle_min);
    lidar_scan->set_angle_increment(angle_increment);
    lidar_scan->set_max_range(max_range);

    // Simulate ranging data based on input.
    // In this example, each sample in "input" is mapped to a range value.
    for (int sample : input) {
        // Transform the integer sample into a float distance measurement (simulation)
        float distance = static_cast<float>(sample) * 0.1f;
        // Ensure the distance doesn't exceed max range
        if (distance > max_range) {
            distance = max_range;
        }
        lidar_scan->add_ranges(distance);
    }

    logger().info("Lidar '{}' generated {} range measurements", name(), lidar_scan->ranges_size());
    return data;
}
