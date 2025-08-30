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
#include "../data/DataClasses.h"
#include "../map/Map2D.h"
#include "../objects/MapObject.h"

class Sensor : public MapObject {
public:
    /**
     * @brief Construct a new Sensor object
     *
     * @param name      Human‑readable identifier
     * @param position  Mounting position (front/back/left/right/…)
     * @param map    Pointer to the map object
     * @param sample_rate_hz Sensor sample rate in Hz
     * @param length Length of the sensor in meters (default: 0.05m)
     * @param width Width of the sensor in meters (default: 0.05m)
     * @param rotation Initial orientation in radians (default: 0)
     * @param init_x Initial x position in meters (default: 0)
     * @param init_y Initial y position in meters (default: 0)
     */
    Sensor(std::string name,
        std::string position,
        Map2D* map,
        uint16_t sample_rate_hz,
        float length = 0.05f,
        float width = 0.05f,
        float rotation = 0.0f,
        float init_x = 0.0f,
        float init_y = 0.0f);

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
    [[nodiscard]] const std::string& positionOnVehicle() const noexcept { return position_on_vehicle_; }

    /// How many Sensor instances currently live in the program
    static std::atomic<int> sensor_count;

    /**
     * @brief Links the sensor to a sensor type
     *
     */
    [[nodiscard]] virtual sensor_data::SensorType sensorType() const = 0;
    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Vehicle; } // Sensors are typically part of vehicles

    void update() override;

protected:
    [[nodiscard]] spdlog::logger& logger() const override { return *rootLogger; }
    Map2D* map_ = nullptr;

private:
    std::string name_;
    std::string position_on_vehicle_;
    uint16_t sample_rate_hz_;
};

#endif //SENSOR_H