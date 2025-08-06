#ifndef VEHICLE_H
#define VEHICLE_H

#include "../data/classes.h"      // for Pose2D
#include "../base/MapObject.h"
#include "../utils/logging.h"    // for logging
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>


// forward declarations, otherwise circular dependency with Sensor.h
class Map2D;
class Sensor;

class Vehicle final : public MapObject {
public:
    /**
     * @brief Construct a Vehicle with given physical properties and initial state.
     * @param init_pose   Initial position (meters) and orientation (radians) of the vehicle.
     * @param length      Vehicle length in meters.
     * @param width       Vehicle width in meters.
     * @param color_bgr   BGR color of the vehicle for visualization (e.g. {255,0,0} for blue).
     * @param init_speed  Initial linear speed in m/s (default 0).
     */
    Vehicle(Pose2D init_pose,
                     double length, double width,
                     std::array<uint8_t,3> color_bgr,
                     double init_speed)
        : MapObject(init_pose, length, width),   // ← initialise the base sub-object
          color_bgr_(color_bgr),
          speed_(init_speed)                     // Vehicle’s own members
    {}

    // Getters
    [[nodiscard]] std::array<uint8_t,3> color() const { return color_bgr_; }
    [[nodiscard]] double speed() const { return speed_; }
    [[nodiscard]] double yawRate() const { return yaw_rate_; }

    // Setters
    // control variables
    void setSpeed(double speed) {speed_ = std::clamp(speed, 0.0, max_speed_);}
    void setSteeringAngle(double angle) {
        steering_angle_ = std::clamp(angle, -max_steering_angle_, max_steering_angle_);
        yaw_rate_ = std::tan(steering_angle_) / length_ * speed_;
    }
    void setSpeedAndSteeringAngle(double speed, double steering_angle) {
        setSpeed(speed);
        setSteeringAngle(steering_angle);
    }
    void setColor(std::array<uint8_t,3> color_bgr) { color_bgr_ = color_bgr; }
    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Vehicle; }

    /** Return the currently attached map or nullptr if detached. */
    [[nodiscard]] Map2D* map() const noexcept { return map_; }; // raw ptr is sufficient because the Map2D class owns its own lifetime

    // Relative position of a sensor on the car body
    enum class MountSide { Front, Back, Left, Right };

    /**
     * @brief Update the vehicle's pose for the current moment. The timestep
     * dt is computed by dividing the duration by the configured FPS (defined in config/constants.h).
     *
     * State variables: x = [x, y, theta]
     * Input: u = [speed, steering_angle]
     *
     * Followings are computed:
     *
     * x_dot = speed * cos(theta)
     * y_dot = speed * sin(theta)
     * yaw_rate = speed * tan(steering_angle) / length
     *
     */
    void update() override;

    /**
     * @brief Add a sensor to the vehicle at a specific mount position.
     * The sensor will be updated with the vehicle's pose when the vehicle moves.
     *
     * @param s Pointer to the Sensor object to mount.
     * @param side Side of the vehicle where the sensor is mounted (Front/Back/Left/Right).
     * @param offset Lateral (Left/Right) or longitudinal (Front/Back) shift from the center [m].
     */
    void addSensor(Sensor* s, MountSide side, double offset = 0.0);

    /**
     * @brief  Register the vehicle as well as its sensors on the map.
     *
     * @param map Pointer to the Map2D object where the vehicle is placed.
     */
    void setMap(Map2D *map) override;

protected:
    /// Override: returns the *type‑local* logger
    [[nodiscard]] spdlog::logger& logger() const override
    {
        // name the logger after the class once – spdlog returns the same ptr next time
        static std::shared_ptr<spdlog::logger> logger_ = utils::getLogger("Vehicle");
        return *logger_;
    }

private:
    struct Mount {
        Sensor*      sensor;
        MountSide    side;
        double       offset;   ///< lateral (Left/Right) or longitudinal (Front/Back) shift [m]
    };
    std::array<uint8_t,3> color_bgr_;   ///< Vehicle color (BGR format)
    double speed_;                      ///< Linear speed (m/s)
    double max_speed_{30.0};         ///< Maximum speed (m/s)
    double yaw_rate_{0.0};                   ///< Angular turn rate (rad/s)
    double steering_angle_ {0.0}; ///< Steering angle (radians), default 0
    double max_steering_angle_ {M_PI/3};
    double accel_ {0.0};              ///< Linear acceleration (m/s^2), default 0
    std::vector<Mount> mounts_;
    /**
     * @brief Push updated poses to mounted sensors as the vehicle moves.
     *
     */
    void updateSensors() const;

    /**
     * @brief Update the vehicle's bounding box footprint.
     */
    // void updateFootprint();
};

#endif // VEHICLE_H
