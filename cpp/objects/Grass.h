#ifndef VEHICLE_SIMULATOR_GRASS_H
#define VEHICLE_SIMULATOR_GRASS_H

#include "../objects/MapObject.h"
#include "../utils/logging.h"

/**
 * @brief Static grass patch. Stamps Cell::Grass into the base map and does not participate in collisions.
 */
class Grass final : public MapObject {
public:
    Grass(float length, float width, float x, float y, float rotation = 0.0f)
        : MapObject(length, width, b2_staticBody, rotation, 0.0f, x, y, 0.0f, 0.8f)
    {
        b2Filter filter = b2DefaultFilter();
        filter.maskBits = 0; // Don't collide with anything
        b2Shape_SetFilter(body_descriptor_.shapeId, filter);
    }

    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Grass; }
    void update() override {}
protected:
    [[nodiscard]] spdlog::logger& logger() const override {
        static std::shared_ptr<spdlog::logger> log = utils::getLogger("Grass");
        return *log;
    }
};

#endif //VEHICLE_SIMULATOR_GRASS_H
