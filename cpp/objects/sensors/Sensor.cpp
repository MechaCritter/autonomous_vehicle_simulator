/**
* @file Sensor.cpp
 * @brief Implementation of the Sensor class
 */
#include "Sensor.h"
#include <utility>

#include "../objects/MapObject.h"
#include "../data/constants.h"

// -------------- ctors / dtors ---------------
Sensor::Sensor(std::string name,
        std::string position,
        Map2D* map,
        uint16_t sample_rate_hz,
        float length,
        float width,
        float rotation,
        float init_x,
        float init_y)
    : MapObject(length, width, b2_dynamicBody, rotation, 0.0f, init_x, init_y, constants::sensors_density, 0.0f),
    name_(std::move(name)),
    position_on_vehicle_(std::move(position)),
    sample_rate_hz_(sample_rate_hz)
{
    map_ = map ? map : &globalMap(); // use the global map if no map provided
    std::cout << name_ << " sensor created at " << position_on_vehicle_ << " with sample rate: " << sample_rate_hz_ << " Hz. \n";
    std::cout << "Global map is used: " << ((map_ == &globalMap()) ? "true" : "false") << "\n";
}

void Sensor::update() {

}

