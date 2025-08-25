#pragma once
#include "../objects/MapObject.h"
#include "utils/logging.h"

class Obstacle final : public MapObject {
public:
    /**
     * @brief Rigid, immovable obstacle (infinite mass, static body).
     */
    Obstacle(float length, float width, float init_x = 0.0f, float init_y = 0.0f, float rotation = 0.0f)
        : MapObject(length, width, b2_staticBody, rotation, 0.0f, init_x, init_y, 1.0f, 0.0f)
    {
        // Static body automatically has infinite mass in Box2D
    }

    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Obstacle; }

    void update() override;
protected:
    [[nodiscard]] spdlog::logger& logger() const override
    {
        static std::shared_ptr<spdlog::logger> logger_ = utils::getLogger("Obstacle");
        return *logger_;
    }
};
