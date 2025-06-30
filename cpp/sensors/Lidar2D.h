//
// Created by vunha on 4/19/2025.
//

#ifndef LIDAR2D_H
#define LIDAR2D_H
#include <opencv2/opencv.hpp>
#include "Sensor.h"
#include "../map/Map2D.h"



class Lidar2D : public Sensor
{
public:
    using Sensor::Sensor;   // inherit the ctor(s)
    [[nodiscard]] sensor_data::SensorType sensorType() const override {
        return sensor_data::SensorType::LIDAR2D;
    }

     /**
     * @brief Cast a ray in the given direction. If it hits an obstacle, return the
     * distance to it.
     *
     * @param rel_angle Relative angle to the lidar's pose in radians. The zero point is the middle point of the fov.
     * @param map The map to cast the ray on.
     * @param cells Optional vector to store the cells that the ray passes through.
     *
     * @return double Distance to the first obstacle in meters. NOTE: in METERS, not pixels!
     */
    [[nodiscard]]
    double castRay(double rel_angle, const Map2D& map, std::vector<std::pair<int,int>>* cells=nullptr) const;

    // Override the generateData function from Sensor.
    [[nodiscard]] std::unique_ptr<sensor_data::SensorData> generateData() override;

    // Getters
    [[nodiscard]] int nBeams() const noexcept { return n_beams_; }
    [[nodiscard]] double fov() const noexcept { return fov_; }
    [[nodiscard]] double maxRange() const noexcept { return max_range_; }
    [[nodiscard]] double angleMin() const noexcept { return angle_min_; }
    [[nodiscard]] double angleIncrement() const noexcept { return angle_increment_; }

private:
    int n_beams_ = 360; // number of beams
    double fov_ = 5 * M_PI / 6; // field of view in radians
    double max_range_ = 10.0; // maximum range in meters
    double angle_min_ = -fov_ / 2.0; // minimum angle in radians
    double angle_increment_ = fov_ / (n_beams_ - 1);

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
