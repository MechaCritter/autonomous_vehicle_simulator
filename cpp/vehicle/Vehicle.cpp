#include "Vehicle.h"
#include "../sensors/Sensor.h"
#include "../config/constants.h"
#include <eigen3/Eigen/Dense>
#include <spdlog/spdlog.h>


void Vehicle::addSensor(Sensor* s, MountSide side, double offset)
{
    mounts_.push_back({s, side, offset});
    updateSensors();   // place immediately
}

void Vehicle::setMap(Map2D *map) {
    MapObject::setMap(map);
    for (auto& m : mounts_) {
        map->addObject(*m.sensor); // register sensor on the map
    }
    updateSensors(); // place sensors immediately
}


void Vehicle::updateSensors() const {
    for (auto& m : mounts_) {
        Eigen::Vector2d rel;          // body-frame offset
        switch (m.side) {
            case MountSide::Front: rel = { length_/2.0 + m.offset, 0.0 }; break;
            case MountSide::Back:  rel = {-length_/2.0 - m.offset, 0.0 }; break;
            case MountSide::Left:  rel = { 0.0,  width_/2.0 + m.offset }; break;
            case MountSide::Right: rel = { 0.0, -width_/2.0 - m.offset }; break;
        }
        // rotate into world frame
        double c = std::cos(pose_.theta), s = std::sin(pose_.theta);
        Eigen::Vector2d world = { pose_.x + c*rel.x() - s*rel.y(),
                                  pose_.y + s*rel.x() + c*rel.y() };
        Pose2D p{ world.x(), world.y(), pose_.theta };
        m.sensor->setPose(p);
        m.sensor->update();
    }
}

void Vehicle::update() {
    updating_ = true;
    const double theta0 = pose_.theta; // save current pose

    /* --- after pose update, move all mounted sensors ------------------- */
    if (std::fabs(yaw_rate_) < 1e-6) {
        // Going straight (no rotation)
        pose_.x += speed_ * std::cos(theta0) * constants::step_size;
        pose_.y += speed_ * std::sin(theta0) * constants::step_size;
        // theta remains the same
    } else {
        // Moving along an arc of constant yaw_rate
        const double theta1 = theta0 + yaw_rate_ * constants::step_size;
        const double R = speed_ / yaw_rate_;  // turning radius
        pose_.x += R * (std::sin(theta1) - std::sin(theta0));
        pose_.y += R * (-std::cos(theta1) + std::cos(theta0));
        pose_.theta = std::fmod(theta1, 2.0 * M_PI); // modulo to avoid theta growing too large
    }
    /* recompute pixel footprint for next Map2D::update() */
    updateFootprint();
    /* --- after pose update, move all mounted sensors ------------------- */
    updateSensors();
}

// void Vehicle::updateFootprint() {
//     footprint_.clear();
//     if (map_ == nullptr) { return;}
//
//     const double half_w = width_  * 0.5;
//     const double half_l = length_ * 0.5;
//
//     /* ---- rectangle corners in body frame (CCW, front-left start) ---- */
//     std::vector<Eigen::Vector2d> corners = {
//                { half_l,  half_w}, { half_l, -half_w},
//                 {-half_l, -half_w}, {-half_l,  half_w}
//             };
//     /* ---- rotate/translate into world frame --------------------------- */
//     auto world = utils::transform(corners, pose_);
//     int min_cx = map_->width(),  min_cy = map_->height();
//     int max_cx = 0,              max_cy = 0;
//     for (auto& p : world)
//     {
//         auto [cx, cy] = map_->worldToCell(p.x(), p.y());
//         min_cx = std::min(min_cx, cx);  max_cx = std::max(max_cx, cx);
//         min_cy = std::min(min_cy, cy);  max_cy = std::max(max_cy, cy);
//     }
//     min_cx = std::clamp(min_cx, 0, map_->width()-1);
//     max_cx = std::clamp(max_cx, 0, map_->width()-1);
//     min_cy = std::clamp(min_cy, 0, map_->height()-1);
//     max_cy = std::clamp(max_cy, 0, map_->height()-1);
//     bbox_ = {min_cx,min_cy,max_cx,max_cy};
//
//     footprint_.reserve(static_cast<std::size_t>((max_cx-min_cx+1)*(max_cy-min_cy+1)));
//     for (int y=min_cy; y<=max_cy; ++y)
//         for (int x=min_cx; x<=max_cx; ++x)
//             if (utils::pointIsInRotatedRechtangle(x, y, map_->resolution(), *this))
//                 footprint_.push_back(map_->cellPtr(x,y));
// }
