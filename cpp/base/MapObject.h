//
// Created by vunha on 8/4/2025.
//

#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "data/classes.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <unordered_map>
#include <vector>

class Map2D;

/*
 * @brief Base class for all objects that can be placed on the map. All objects that can move
 * on the map has to inherit from this class.
 *
 */
class MapObject {

public:
    /* Pointer alias for direct cell access. */
    using CellPtr = Cell*;

    /* constructor. Upon creation, registers this object on the map */
    MapObject(Pose2D pose, double length, double width)
    : pose_(pose), length_(length), width_(width) {}

    virtual ~MapObject() = default;

    // getters
    [[nodiscard]] const Pose2D& pose() const  { return pose_; }
    [[nodiscard]] double length() const { return length_; }
    [[nodiscard]] double width()  const { return width_; }
    [[nodiscard]] bool isUpdating() const noexcept { return updating_; }

    /**
     * Allows the object to be started updating its position continuously.
     */
    void requestStartUpdate() noexcept { updating_ = true; }

    /**
     * Sets a flag to stop updating the object's position.
     */
    void requestStopUpdate() noexcept { updating_ = false; }

    /* Returns the currently occupied cells. */
    [[nodiscard]] const std::vector<CellPtr>& footprint() const noexcept { return footprint_; }

    /* The cell type associated with this object. */
    [[nodiscard]] virtual Cell cellType() const noexcept = 0;

    /** Update the object's current state for the current time step.
     */
    virtual void update() = 0;

    /** Update the object's current position continuously for the defined period
     * or until the flag `updating_` is set to false.
     *
     * @param duration Total update time in seconds.
     */
    void updateFor(unsigned int duration);

    /**
     * Update the object's current position continuously until the flag
     * `updating_` is set to false.
     */
    void updateForever();

    /** Attach this object to a map so it can read/modify it.
     *
     * @param map: the map to attach the object to
     */
    virtual void setMap(Map2D* map) { this->map_ = map; }

    /** @brief Update the object’s world pose.
     *
     * @param p The new pose of the object in the world frame.
     */
    void setPose(const Pose2D& p) { pose_ = p; }
protected:
    /// **Every derived class overrides this and returns its own logger.**
    [[nodiscard]] virtual spdlog::logger& logger() const = 0;
    Pose2D pose_{0, 0, 0}; ///< Position (meters) and heading (radians)
    double length_{1}, width_{1}; ///< Physical dimensions in meters
    bool updating_{false}; ///< Whether the object is currently updating its position
    Map2D* map_{nullptr};
    std::vector<CellPtr> footprint_;   ///< Cached addresses of occupied cells
    struct BoundingBox { int min_x, min_y, max_x, max_y; } bbox_{0,0,0,0};
    /**
     * Update the object's current footprint on the map for the current time step.
    */
    void updateFootprint();
};



#endif //MAPOBJECT_H
