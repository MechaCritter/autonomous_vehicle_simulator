#include "Lidar2D.h"
#include "../utils/utils.h"
#include <chrono>

// constants
std::filesystem::path lidar_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";


double Lidar2D::castRay(double rel_angle, const Map2D &map, std::vector<std::pair<int,int>>* cells) const {
    const double step = 0.5 * map.resolution();
    const b2Vec2 pos = position();
    const float theta = b2Rot_GetAngle(rotation());
    const double global_angle = theta + rel_angle;
    const double half_fov = 0.5 * fov_;
    const auto map_snapshot = map.snapshot();
    if (map_snapshot.size() != static_cast<size_t>(map.width() * map.height())) {
        throw std::runtime_error("Map snapshot size mismatch. Actual: " + std::to_string(map_snapshot.size()) +
                                 ", Expected: " + std::to_string(static_cast<size_t>(map.width() * map.height())) + ".");
    }
    if (rel_angle > half_fov || rel_angle < -half_fov) {
        throw std::invalid_argument("Angle out of range: " + std::to_string(rel_angle) +
                                    " (max: " + std::to_string(half_fov) + ", min: " + std::to_string(-half_fov) + ")");}

    /* start one step away to avoid hitting the vehicle cell itself */
    for (double d = step; d <= max_range_; d += step) {

        const double world_x = pos.x + d * cos(global_angle);
        const double world_y = pos.y + d * sin(global_angle);

        auto [pixel_x, pixel_y] = map.worldToCell(world_x, world_y); // convert to pixel coordinates

        // if px and py are out of bounds, assume no obstacle => return max range
        if (pixel_x < 0 || pixel_x >= map.width() || pixel_y < 0 || pixel_y >= map.height()) {
            return max_range_;
        }

        if (cells) cells->emplace_back(pixel_x, pixel_y); // record path

        const Cell c = map_snapshot[pixel_y * map.width() + pixel_x];
        if (c == Cell::Obstacle || c == Cell::Vehicle || c == Cell::Reserved) {
            logger().info("Lidar {} hit obstacle {} at ({},{})",
                           name(), Map2D::cellToString(c), pixel_x, pixel_y);
            return d;
        }
    }
    return max_range_; // hit nothing
}

std::unique_ptr<sensor_data::SensorData> Lidar2D::generateData()
{
    if (!map_) throw std::runtime_error("Lidar map pointer not set!");

    auto data = std::make_unique<sensor_data::SensorData>();

    auto* hdr = data->mutable_header();
    hdr->set_name(name());
    hdr->set_sensor_type(sensorType());
    auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now()).time_since_epoch().count();
    hdr->set_timestamp(static_cast<uint64_t>(now));

    auto* scan = data->mutable_lidar_scan_2d();
    scan->set_angle_min(angle_min_);
    scan->set_angle_increment(angle_increment_);
    scan->set_max_range(max_range_);

    std::ostringstream oss;

    for (int i = 0; i < n_beams_; ++i) {
        double angle = angle_min_ + i * angle_increment_;
        double range = castRay(angle, *map_, nullptr); // no cells needed for basic operation
        scan->add_ranges(range);
        oss << range << (i+1 == n_beams_ ? "" : ", ");
    }

    logger().debug("Lidar {} produced scan [{}]", name(), oss.str());
    return data;
}

#ifdef WITH_OPENCV_DEBUG
std::unique_ptr<sensor_data::SensorData> Lidar2D::generateDataWithDebugVideo(const std::string& video_filename)
{
    if (!map_) throw std::runtime_error("Lidar map pointer not set!");

    auto data = std::make_unique<sensor_data::SensorData>();

    auto* hdr = data->mutable_header();
    hdr->set_name(name());
    hdr->set_sensor_type(sensorType());
    auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now()).time_since_epoch().count();
    hdr->set_timestamp(static_cast<uint64_t>(now));

    auto* scan = data->mutable_lidar_scan_2d();
    scan->set_angle_min(angle_min_);
    scan->set_angle_increment(angle_increment_);
    scan->set_max_range(max_range_);

    std::ostringstream oss;

    // Start motion capture for debug video
    map_->startSimulation();

    for (int i = 0; i < n_beams_; ++i) {
        double angle = angle_min_ + i * angle_increment_;

        std::vector<std::pair<int,int>> cells;
        double range = castRay(angle, *map_, &cells); // get cells hit by the ray
        cv::Mat frame = map_->renderToMat();

        /* yellow 3×3 square at lidar location */
        const b2Vec2 pos = position();
        int cx = static_cast<int>(pos.x / map_->resolution());
        int cy = static_cast<int>(pos.y / map_->resolution());
        cv::rectangle(frame,
                      cv::Point(cx-1, cy-1), cv::Point(cx+1, cy+1),
                      cv::Scalar(0,255,255), cv::FILLED);

        /* red beam pixels */
        for (auto [x, y] : cells)
            frame.at<cv::Vec3b>(y, x) = {0,0,255};

        /* distance text under hit-cell (only if we really hit) */
        if (range < max_range_ && !cells.empty()) {
            auto [hx, hy] = cells.back();
            cv::putText(frame,
                        std::to_string(range),                // metres
                        cv::Point(hx, hy),
                        cv::FONT_HERSHEY_SIMPLEX, 0.15,
                        cv::Scalar(255,255,255), 1, cv::LINE_AA);
        }

        map_->addMotionFrame(frame);      // one frame per beam

        scan->add_ranges(range);
        oss << range << (i+1 == n_beams_ ? "" : ", ");
    }

    // End simulation and save video
    map_->endSimulation();
    map_->flushFrames(video_filename);

    logger().debug("Lidar {} produced scan [{}] with debug video saved to {}", name(), oss.str(), video_filename);
    return data;
}
#endif
