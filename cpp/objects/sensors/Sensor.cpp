/**
* @file Sensor.cpp
 * @brief Implementation of the Sensor class
 */
#include "Sensor.h"
#include <utility>
#include <atomic>

#include "../objects/MapObject.h"
#include "../vehicle/Vehicle.h"

// Define static member
std::atomic<int> Sensor::sensor_count{0};

// -------------- ctors / dtors ---------------
Sensor::Sensor(std::string name,
        std::string position,
        uint16_t sample_rate_hz,
        float length,
        float width,
        float rotation,
        float init_x,
        float init_y)
    : MapObject(length, width, b2_dynamicBody, rotation, 0.0f, init_x, init_y, 0.0f, 0.0f),
    name_(std::move(name)),
    position_on_vehicle_(std::move(position)),
    sample_rate_hz_(sample_rate_hz)
{
    b2Filter filter = b2DefaultFilter();
    filter.maskBits = 0; // Don't collide with anything
    b2Shape_SetFilter(body_descriptor_.shapeId, filter);
}

void Sensor::update() {

}

void Sensor::setOwner(Vehicle* owner) noexcept
{
    owner_ = owner;
    owner_body_id_ = owner->bodyId();
    owner_filter_ = owner->bodyFilter();
}
