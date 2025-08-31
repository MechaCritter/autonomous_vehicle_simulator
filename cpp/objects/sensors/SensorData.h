#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <vector>
#include <string>
#include <memory>
#include <cstdint>

namespace sensor_data {

enum class SensorType : int {
    LIDAR2D = 0,
    ODOMETER = 1,
    IMU = 2,
    GPS = 3,
    CAMERA2D = 4
};

struct Header {
    std::string name;
    SensorType sensor_type{SensorType::LIDAR2D};
    std::uint64_t timestamp{0};

    // Protobuf-like setters for compatibility
    void set_name(const std::string& n) { name = n; }
    void set_sensor_type(SensorType t) { sensor_type = t; }
    void set_timestamp(std::uint64_t ts) { timestamp = ts; }
};

struct LidarScan2D {
    double angle_min_{0.0};
    double angle_increment_{0.0};
    double max_range_{0.0};
    std::vector<double> ranges_;

    // Protobuf-like setters for compatibility
    void set_angle_min(double v) { angle_min_ = v; }
    void set_angle_increment(double v) { angle_increment_ = v; }
    void set_max_range(double v) { max_range_ = v; }
    void add_ranges(double v) { ranges_.push_back(v); }

    // Protobuf-like getters for compatibility
    double angle_min() const { return angle_min_; }
    double angle_increment() const { return angle_increment_; }
    double max_range() const { return max_range_; }
    int ranges_size() const { return static_cast<int>(ranges_.size()); }
    double ranges(int i) const { return ranges_[static_cast<size_t>(i)]; }
};

struct SensorData {
    Header header_;
    LidarScan2D lidar_scan_2d_;

    // Protobuf-like accessors for compatibility
    bool has_lidar_scan_2d() const { return true; }
    LidarScan2D* mutable_lidar_scan_2d() { return &lidar_scan_2d_; }
    LidarScan2D& lidar_scan_2d() { return lidar_scan_2d_; }
    const LidarScan2D& lidar_scan_2d() const { return lidar_scan_2d_; }

    Header* mutable_header() { return &header_; }
    Header& header() { return header_; }
    const Header& header() const { return header_; }
};

} // namespace sensor_data

#endif // SENSOR_DATA_H
