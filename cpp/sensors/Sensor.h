/**
 * @file Sensor.h
 * This module includes the base class for all sensor modules.  
 * All specific sensor implementations should inherit from the base Sensor class
 * defined in this file.
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <string>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "proto/generated/sensor_data.pb.h"
#include "../data/classes.h"

class Sensor {
public:
    /**
     * @brief Construct a new Sensor object
     *
     * @param name      Human‑readable identifier
     * @param position  Mounting position (front/back/left/right/…)
     * @param sample_rate_hz Sensor sample rate in Hz
     */
    Sensor(std::string  name,
        std::string   position,
        uint16_t     sample_rate_hz = 10,
        Pose2D pose = {0, 0, 0});

    /**
     * @brief Destructor – keeps the global sensor counter honest.
     */
    virtual ~Sensor();

    Sensor(const Sensor&)            = default; // called when "Sensor s2 = s1;"
    Sensor& operator=(const Sensor&) = default; // called when "s2 = s1;"
    Sensor(Sensor&&) noexcept        = default; // called when "Sensor s2 = std::move(s1);"
    Sensor& operator=(Sensor&&)      = default; // called when "s2 = std::move(s1);"

    /**
     * @brief Generate some data from an input sample buffer.
     *
     * Sums the input values. Meant as a placeholder for
     * real sensor processing.
     *
     * @param input A vector of sample integers
     * @return int  Sum of all samples (32‑bit; promoted internally)
     */
    [[nodiscard]] virtual std::unique_ptr<sensor_data::SensorData> generateData(const std::vector<int>& input) = 0;

    [[nodiscard]] const std::string& name()     const noexcept { return name_; }
    [[nodiscard]] const std::string& position() const noexcept { return position_; }

    /// How many Sensor instances currently live in the program
    static std::atomic<int> sensor_count;

    /**
     * @brief Links the sensor to a sensor type
     *
     */
    [[nodiscard]] virtual sensor_data::SensorType protoType() const = 0;

private:
    /// **Every derived class overrides this and returns its own logger.**
    [[nodiscard]] virtual spdlog::logger& logger() const = 0;

    std::string name_;
    std::string position_;
    uint8_t  id_; // unique ID for this sensor instance
    uint16_t sample_rate_hz_;
    Pose2D pose_; //TODO: pose has to be updated as the vehicle moves
};

#endif //SENSOR_H