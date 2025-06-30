/**
* @file Sensor.cpp
 * @brief Implementation of the Sensor class
 */
#include "Sensor.h"
#include <utility>

#include "../utils/utils.h"

// -------------- static data -----------------
std::atomic<int> Sensor::sensor_count{0};

// -------------- ctors / dtors ---------------
Sensor::Sensor(std::string  name, std::string position, Map2D* map, uint16_t sample_rate_hz, Pose2D pose)
    : name_{std::move(name)},
      position_{std::move(position)},
      sample_rate_hz_{sample_rate_hz},
      pose_{pose},
    map_{ map ? map : &globalMap() }

{
    std::cout << name_ << " sensor created at " << position_ << " with sample rate: " << sample_rate_hz_ << " Hz. ";
    std::cout << "Global map is used: " << ((map_ == &globalMap()) ? "true" : "false") << std::endl;
    id_ = sensor_count.fetch_add(1, std::memory_order_relaxed);
}

Sensor::~Sensor()
{
    sensor_count.fetch_sub(1, std::memory_order_relaxed);
}


