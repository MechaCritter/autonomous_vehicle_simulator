/**
 * @file Sensor.h
 * This module includes the base class for all sensor modules.  
 * All specific sensor implementations should inherit from the base Sensor class
 * defined in this file.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include "proto/generated/sensor_data.pb.h"
#include "../data/classes.h"
#include "../map/Map2D.h"
#include "../base/MapObject.h"

class Sensor : public MapObject {
public:
    /**
     * @brief Construct a new Sensor object
     *
     * @param name      Human‑readable identifier
     * @param position  Mounting position (front/back/left/right/…)
     * @param map    Pointer to the map object
     * @param sample_rate_hz Sensor sample rate in Hz
     * @param pose   Pose of the sensor in the map
     */
    Sensor(const std::string &name,
        const std::string &position,
        Map2D* map,
        uint16_t sample_rate_hz,
        Pose2D pose);

    Sensor(const Sensor&)            = default; // called when "Sensor s2 = s1;"
    Sensor& operator=(const Sensor&) = default; // called when "s2 = s1;"

    Sensor(Sensor&&) noexcept        = default; // called when "Sensor s2 = std::move(s1);"
    Sensor& operator=(Sensor&&)      = default; // called when "s2 = std::move(s1);"

    ~Sensor();

    /**
     * @brief Generate some data from an input sample buffer.
     *
     * Sums the input values. Meant as a placeholder for
     * real sensor processing.
     *
     * @return int  Sum of all samples (32‑bit; promoted internally)
     */
    [[nodiscard]] virtual std::unique_ptr<sensor_data::SensorData> generateData() = 0;

    [[nodiscard]] const std::string& name()     const noexcept { return name_; }
    [[nodiscard]] const std::string& position() const noexcept { return position_; }

    /// How many Sensor instances currently live in the program
    static std::atomic<int> sensor_count;

    /**
     * @brief Links the sensor to a sensor type
     *
     */
    [[nodiscard]] virtual sensor_data::SensorType sensorType() const = 0;


protected:
    [[nodiscard]] uint8_t id() const noexcept { return id_;  }
    Map2D* map_ = nullptr;

private:
    std::string name_;
    std::string position_;
    uint8_t  id_; // unique ID for this sensor instance
    uint16_t sample_rate_hz_;
    int length_ = 2;
    int width_ = 2;
};

#endif //SENSOR_H