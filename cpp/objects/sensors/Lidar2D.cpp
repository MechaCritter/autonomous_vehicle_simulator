#include "Lidar2D.h"
#include "../utils/utils.h"
#include <chrono>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <vector>
#include <iostream>

// constants
std::filesystem::path lidar_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

// Small POD to carry results back from the ray-cast callback
struct RaycastCtx {
    b2BodyId ignoreA{};
    b2BodyId ignoreB{};
    float    bestFraction{1.0f};
    bool     hit{false};
};

// Ray-cast callback: skip owner + this sensor; skip sensors; accept first valid hit
static float LidarRaycastCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* user)
{
    auto* ctx = static_cast<RaycastCtx*>(user);
    const b2BodyId body = b2Shape_GetBody(shapeId);
    if (B2_ID_EQUALS(body, ctx->ignoreA) || B2_ID_EQUALS(body, ctx->ignoreB)) {
        return -1.0f; // ignore and continue
    }
    if (b2Shape_IsSensor(shapeId)) {
        return -1.0f; // sensors never block the beam
    }
    // accept this hit and clip the ray to this point (closest-so-far)
    ctx->hit = true;
    ctx->bestFraction = fraction;
    return fraction;
}

double Lidar2D::castRay(double rel_angle) const {
    // const double step = 0.5 * map.resolution();
    const b2Vec2 origin = position();
    const float theta = b2Rot_GetAngle(rotation());
    const double global_angle = theta + rel_angle;
    const double half_fov = 0.5 * fov_;
    if (rel_angle > half_fov || rel_angle < -half_fov) {
        throw std::invalid_argument("Angle out of range: " + std::to_string(rel_angle) +
                                    " (max: " + std::to_string(half_fov) + ", min: " + std::to_string(-half_fov) + ")");}

    const b2Vec2 direction   = { static_cast<float>(std::cos(global_angle)),
                       static_cast<float>(std::sin(global_angle)) };
    const b2Vec2 translation = { direction.x * static_cast<float>(max_range_),
                             direction.y * static_cast<float>(max_range_) };

    b2QueryFilter qf = b2DefaultQueryFilter();
    if (owner()) {
        const b2Filter of = ownerFilter();
        qf.categoryBits = of.categoryBits;
        qf.maskBits     = of.maskBits;
    }

    RaycastCtx ctx{};
    ctx.ignoreA = body_descriptor_.bodyId;
    ctx.ignoreB = owner() ? ownerBodyId() : b2BodyId{};
    castRayWorld(origin, translation, qf, &LidarRaycastCallback, &ctx);

    const double d = ctx.hit ? (ctx.bestFraction * max_range_) : max_range_;

    // /* start one step away to avoid hitting the vehicle cell itself */
    // for (double d = step; d <= max_range_; d += step) {
    //
    //     const double world_x = origin.x + d * cos(global_angle);
    //     const double world_y = origin.y + d * sin(global_angle);
    //
    //     auto [pixel_x, pixel_y] = map.worldToCell(world_x, world_y); // convert to pixel coordinates
    //
    //     // if px and py are out of bounds, assume no obstacle => return max range
    //     if (pixel_x < 0 || pixel_x >= map.width() || pixel_y < 0 || pixel_y >= map.height()) {
    //         return max_range_;
    //     }
    //
    //     if (cells) cells->emplace_back(pixel_x, pixel_y); // record path
    //
    //     const Cell c = map_snapshot[pixel_y * map.width() + pixel_x];
    //     if ((c == Cell::Obstacle || c == Cell::Vehicle || c == Cell::Reserved)) {
    //         logger().info("Lidar {} hit obstacle {} at ({},{})",
    //                        name(), Map2D::cellToString(c), pixel_x, pixel_y);
    //         return d;
    //     }
    // }
    return d;; // hit nothing
}

std::unique_ptr<sensor_data::SensorData> Lidar2D::generateData()
{
    if (!map_) throw std::runtime_error("Lidar map pointer not set!");

    auto data = std::make_unique<sensor_data::SensorData>();

    auto* hdr = data->mutable_header();
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
        double range = castRay(angle); // no cells needed for basic operation
        scan->add_ranges(range);
        oss << range << (i+1 == n_beams_ ? "" : ", ");
    }

    logger().debug("Lidar {} produced scan [{}]", name(), oss.str());
    return data;
}

#ifdef WITH_OPENCV_DEBUG
std::unique_ptr<sensor_data::SensorData> Lidar2D::generateDataWithDebugVideo(const std::string& video_filename) const {
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

        std::vector<std::pair<int,int>> cells;
        double range = castRay(angle); // get cells hit by the ray
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
    map_->flushFrames(video_filename);

    logger().debug("Lidar {} produced scan [{}] with debug video saved to {}", name(), oss.str(), video_filename);
    return data;
}
#endif
