/**
* @file Sensor.cpp
 * @brief Implementation of the Sensor class
 */
#include "Sensor.h"
#include <utility>

#include "../base/MapObject.h"
#include "../config/constants.h"

// -------------- ctors / dtors ---------------
Sensor::Sensor(std::string name,
        std::string position,
        Map2D* map = nullptr,
        uint16_t sample_rate_hz = 10,
        Pose2D pose = {0, 0, 0})
    : MapObject(pose, 5, 5),
    map_(map ? map : &globalMap()),
    name_(std::move(name)),
    position_(std::move(position)),
    sample_rate_hz_(sample_rate_hz) // use the global map if no map
{
    // Calculate mass based on sensor dimensions and density
    mass_ = length_ * width_ * constants::sensors_density;
    std::cout << name_ << " sensor created at " << position_ << " with sample rate: " << sample_rate_hz_ << " Hz. \n";
    std::cout << "Global map is used: " << ((map_ == &globalMap()) ? "true" : "false") << "\n";
}

