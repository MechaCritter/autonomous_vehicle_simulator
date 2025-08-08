//
// Created by vunha on 8/4/2025.
//

#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "data/classes.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <unordered_map>
#include <bitset>
#include <queue>
#include <mutex>
#include <vector>

class Map2D;

inline std::shared_ptr<spdlog::logger> rootLogger = spdlog::stdout_color_mt("root");

/**
 * @brief Pool for 16-bit IDs with recycling and thread safety.
 *
 * - allocate(): returns a free ID or throws if none available.
 * - release(id): returns the ID to the pool; never throws.
 *
 * Invariants:
 * - Each allocated ID is marked in-use; double-allocation is detected.
 * - Double-release is ignored (you can log it if you like).
 */
class IdPool16 {
public:
    /** Allocate an ID, reusing freed IDs first; throws if exhausted. */
    uint16_t allocate() {
        std::lock_guard<std::mutex> lock(m_);
        uint16_t id;
        if (!free_.empty()) {
            id = free_.front();
            free_.pop();
        } else if (next_ <= std::numeric_limits<uint16_t>::max()) {
            id = static_cast<uint16_t>(next_++);
        } else {
            throw std::runtime_error("No more uint16_t IDs available (65536 in use).");
        }
        if (in_use_.test(id)) {
            // Should never happen; defensive check.
            throw std::logic_error("ID reuse detected (double-allocation).");
        }
        in_use_.set(id, true);
        return id;
    }

    /** Return an ID to the pool; never throws (safe for destructors).
     *
     * @param id The ID to release.
     */
    void release(uint16_t id) noexcept {
        std::lock_guard<std::mutex> lock(m_);
        if (!in_use_.test(id)) {
            // Double free or release of an ID that isn't owned.
            rootLogger->warn("Attempted to release an ID that is not in use: {}", id);
            return;
        }
        in_use_.set(id, false);
        free_.push(id);
    }

    /** Optional: number of IDs currently in use (O(n) in bitset count). */
    [[nodiscard]]
    std::size_t num_ids_currently_used() const noexcept {
        std::lock_guard<std::mutex> lock(m_);
        return in_use_.count();
    }

private:
    mutable std::mutex m_;
    std::queue<uint16_t> free_;
    std::bitset<65536> in_use_{};
    uint32_t next_{0};
};

/*
 * @brief Base class for all objects that can be placed on the map. All objects that can move
 * on the map has to inherit from this class.
 *
 */
class MapObject {

public:
    /* Pointer alias for direct cell access. */
    using CellPtr = Cell*;

    /** constructor. Upon creation, registers this object on the map */
    MapObject(Pose2D pose, double length, double width):
                id_(s_pool.allocate()), owns_id_(true),
                pose_(pose),
                length_(length),
                width_(width) {
        rootLogger->info("MapObject created with ID: {}", id_);
    }

    // Non-copyable (identity should be unique).
    MapObject(const MapObject&)            = delete;
    MapObject& operator=(const MapObject&) = delete;
    // Movable (transfer ID ownership).
    MapObject(MapObject&& other) noexcept
        : id_(other.id_), owns_id_(other.owns_id_) {
        other.owns_id_ = false;
    }

    MapObject& operator=(MapObject&& other) noexcept {
        if (this != &other) {
            if (owns_id_) {
                s_pool.release(id_);
            }
            id_       = other.id_;
            owns_id_  = other.owns_id_;
            other.owns_id_ = false;
        }
        return *this;
    }

    virtual ~MapObject() {
        if (owns_id_) {
            s_pool.release(id_);
            rootLogger->info("MapObject with ID: {} destroyed", id_);
        }
    }

    // getters
    [[nodiscard]] const Pose2D& pose() const  { return pose_; }
    [[nodiscard]] double length() const { return length_; }
    [[nodiscard]] double width()  const { return width_; }
    [[nodiscard]] double mass() const { return mass_; }
    [[nodiscard]] bool isUpdating() const noexcept { return updating_; }
    [[nodiscard]] uint16_t id() const noexcept { return id_;  }

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
    static IdPool16 s_pool; // pool of ids
    uint16_t id_; // unique ID for this sensor instance
    bool owns_id_{false};
    Pose2D pose_{0, 0, 0}; ///< Position (meters) and heading (radians)
    double length_{1}, width_{1}; ///< Physical dimensions in meters
    double mass_{1}; ///< Mass of the object in kg
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
