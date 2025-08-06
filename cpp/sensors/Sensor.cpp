/**
* @file Sensor.cpp
 * @brief Implementation of the Sensor class
 */
#include "Sensor.h"
#include <utility>

#include "../utils/utils.h"
#include "../base/MapObject.h"

// -------------- static data -----------------
std::atomic<int> Sensor::sensor_count{0};

// -------------- ctors / dtors ---------------
Sensor::Sensor(const std::string &name,
        const std::string &position,
        Map2D* map = nullptr,
        uint16_t sample_rate_hz = 10,
        Pose2D pose = {0, 0, 0})
    : MapObject(pose, 5, 5),
    id_(0),
    sample_rate_hz_(sample_rate_hz),
    name_(name),
    position_(position),
    map_(map ? map : &globalMap()) // use the global map if no map
{
    std::cout << name_ << " sensor created at " << position_ << " with sample rate: " << sample_rate_hz_ << " Hz. \n";
    std::cout << "Global map is used: " << ((map_ == &globalMap()) ? "true" : "false") << "\n";
    id_ = sensor_count.fetch_add(1, std::memory_order_relaxed);
}

Sensor::~Sensor()
{
    sensor_count.fetch_sub(1, std::memory_order_relaxed);
}


