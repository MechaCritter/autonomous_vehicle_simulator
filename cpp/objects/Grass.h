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
        // Make shape a sensor => no collision response
        body_descriptor_.shape_def.isSensor = true;
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
