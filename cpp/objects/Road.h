#ifndef VEHICLE_SIMULATOR_ROAD_H
#define VEHICLE_SIMULATOR_ROAD_H

#include "../objects/MapObject.h"
#include "../utils/logging.h"

/**
 * @brief Static road patch. Stamps Cell::Road into the base map and does not participate in collisions.
 */
class Road final : public MapObject {
public:
    Road(float length, float width, float x, float y, float rotation = 0.0f)
        : MapObject(length, width, b2_staticBody, rotation, 0.0f, x, y, 0.0f, constants::vehicle_road_friction) // density ignored for static
    {
        // Make shape a sensor => no collision response
        body_descriptor_.shape_def.isSensor = true;
    }

    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Road; }
    void update() override {} // no-op
protected:
    [[nodiscard]] spdlog::logger& logger() const override {
        static std::shared_ptr<spdlog::logger> log = utils::getLogger("Road");
        return *log;
    }
};

#endif //VEHICLE_SIMULATOR_ROAD_H
