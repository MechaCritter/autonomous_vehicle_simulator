#pragma once

#include "../objects/MapObject.h"

/**
 * @brief Free space object. No friction, no collision whatsoever.
 * This represents empty/free space in the simulation.
 */
class Free final : public MapObject {
    /**
     * @brief Construct a new Free object
     *
     * @param length Length of the free area in meters
     * @param width Width of the free area in meters
     * @param init_x Initial x position in world coordinates (meters)
     * @param init_y Initial y position in world coordinates (meters)
     * @param rotation Initial rotation in radians
     */
public:
    Free(float length, float width, float init_x = 0.0f, float init_y = 0.0f, float rotation = 0.0f)
        : MapObject(length, width, b2_staticBody, rotation, 0.0f, init_x, init_y, 0.0f, 0.0f) // No density, no friction
    {
        // Set collision filter to not collide with anything
        b2Filter filter = b2DefaultFilter();
        filter.maskBits = 0; // Don't collide with anything
        filter.categoryBits = 0; // Don't belong to any category
        b2Shape_SetFilter(body_descriptor_.shapeId, filter);
    }

    /**
     * @brief Get the cell type for this object
     * @return Cell::Free
     */
    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Free; }

    /**
     * @brief Update method - Free objects don't need to update anything
     */
    void update() override {
        // Free objects are static and don't need updates
    }

protected:
    [[nodiscard]] spdlog::logger& logger() const override {
        static std::shared_ptr<spdlog::logger> logger_ = spdlog::stdout_color_mt("Free");
        return *logger_;
    }
};

