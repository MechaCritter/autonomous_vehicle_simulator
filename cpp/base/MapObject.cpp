//
// Created by vunha on 8/4/2025.
//

#include "MapObject.h"
#include "map/Map2D.h"
#include "config/constants.h"


void MapObject::updateFor(unsigned int duration) {
    requestStartUpdate();
    for (double t = 0.0; t < duration && updating_; t += constants::step_size) {
        update();
        std::this_thread::sleep_for(std::chrono::duration<float>(constants::step_size));
    }
}

void MapObject::updateForever() {
    requestStartUpdate();
    while (updating_) {
        update();
        std::this_thread::sleep_for(std::chrono::duration<float>(constants::step_size));
    }
}

void MapObject::updateFootprint() {
    footprint_.clear();
    if (map_ == nullptr) { return;}

    const double half_w = width_  * 0.5;
    const double half_l = length_ * 0.5;

    /* ---- rectangle corners in body frame (CCW, front-left start) ---- */
    std::vector<Eigen::Vector2d> corners = {
        { half_l,  half_w}, { half_l, -half_w},
         {-half_l, -half_w}, {-half_l,  half_w}
    };
    /* ---- rotate/translate into world frame --------------------------- */
    auto world = utils::transform(corners, pose_);
    int min_cx = map_->width(),  min_cy = map_->height();
    int max_cx = 0,              max_cy = 0;
    for (auto& p : world)
    {
        auto [cx, cy] = map_->worldToCell(p.x(), p.y());
        min_cx = std::min(min_cx, cx);  max_cx = std::max(max_cx, cx);
        min_cy = std::min(min_cy, cy);  max_cy = std::max(max_cy, cy);
    }
    min_cx = std::clamp(min_cx, 0, map_->width()-1);
    max_cx = std::clamp(max_cx, 0, map_->width()-1);
    min_cy = std::clamp(min_cy, 0, map_->height()-1);
    max_cy = std::clamp(max_cy, 0, map_->height()-1);
    bbox_ = {min_cx,min_cy,max_cx,max_cy};

    footprint_.reserve(static_cast<std::size_t>((max_cx-min_cx+1)*(max_cy-min_cy+1)));
    for (int y=min_cy; y<=max_cy; ++y)
        for (int x=min_cx; x<=max_cx; ++x)
            if (utils::pointIsInRotatedRechtangle(x, y, map_->resolution(), *this))
                footprint_.push_back(map_->cellPtr(x,y));
}
