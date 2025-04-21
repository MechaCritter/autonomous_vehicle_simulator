//
// Created by vunha on 4/19/2025.
//

#ifndef LIDAR2D_H
#define LIDAR2D_H
#include "Sensor.h"
#include "../map/Map2D.h"



class Lidar2D : public Sensor
{
public:
    using Sensor::Sensor;   // inherit the ctor(s)

    [[nodiscard]] sensor_data::SensorType protoType() const override {
        return sensor_data::SensorType::LIDAR2D;
    }

     /**
     * @brief Cast a ray in the given direction. If it hits an obstacle, return the
     * distance to it.
     *
     * /TODO: the second arg should be a Map2D& or similar object, from which the position of the
     * /TODO obstacle can be determined.
     *
     * @param angle Angle in radians
     * @return double Distance to the first obstacle in meters
     */
    [[nodiscard]]
    double castRay(double angle, Map2D map);

    // Override the generateData function from Sensor.
    [[nodiscard]]
    std::unique_ptr<sensor_data::SensorData> generateData(const std::vector<int>& input) override;


protected:
    /// Override: returns the *type‑local* logger
    [[nodiscard]] spdlog::logger& logger() const override
    {
        // name the logger after the class once – spdlog returns the same ptr next time
        static std::shared_ptr<spdlog::logger> logger_ =
            spdlog::stdout_color_mt("lidar");
        return *logger_;
    }
};


#endif //LIDAR2D_H
