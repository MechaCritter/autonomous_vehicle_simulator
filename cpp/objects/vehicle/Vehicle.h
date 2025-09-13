#ifndef VEHICLE_H
#define VEHICLE_H

#include "../MapObject.h"
#include "../../utils/logging.h"
#include "../../data/constants.h"
#include <../objects/vehicle/PIDController.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>
#include <limits>
#include <box2d/box2d.h>


// forward declarations, otherwise circular dependency with Sensor.h
class Sensor;
class Map2D;

/**
 * @note the vehicle owns its own sensors instead of transfering to the map! so destroying the
 * vehicle also destroys the sensors.
 */
class Vehicle final : public MapObject {
public:
    /**
     * @brief Construct a Vehicle with given physical properties and initial state. Upon initializing, the vehicle will
     * go to "drive" mode.
     *
     * @note the steering angle will be set to be equal to the initial rotation after initialization.
     *
     * @param length      Vehicle length in meters.
     * @param width       Vehicle width in meters.
     * @param color_bgr   BGR color of the vehicle for visualization (e.g. {255,0,0} for blue).
     * @param rotation    Initial orientation in radians (default: 0).
     * @param init_speed  Initial linear speed in m/s (default 0).
     * @param init_x      Initial x position in meters (default: 0).
     * @param init_y      Initial y position in meters (default: 0).
     * @param motor_force Initial motor force in Newtons (default: 0).
     */
    Vehicle(float length, float width,
            std::array<uint8_t,3> color_bgr = {255,0,0},
            float rotation = 0.0f,
            float init_speed = 0.0f,
            float init_x = 0.0f,
            float init_y = 0.0f,
            float motor_force = 0.0f);

    ~Vehicle() override; // reimplemented the destructor here to avoid a weird error with unique pointer (i still have no idea why)

    // Operational mode of the vehicle
    enum class Mode { Drive, Rest, Brake };

    // Getters
    [[nodiscard]] std::array<uint8_t,3> color() const { return color_bgr_; }
    [[nodiscard]] b2Vec2 velocityVector() const noexcept {
        return b2Body_GetLinearVelocity(body_descriptor_.bodyId);
    }
    [[nodiscard]] float speed() const { return b2Length(velocityVector()); }
    [[nodiscard]] const std::vector<std::unique_ptr<Sensor>>& sensors() const noexcept {
        return sensors_;
    }
    [[nodiscard]] float yawRate() const { return yaw_rate_; }
    [[nodiscard]] float motorForce() const { return motor_force_; }
    [[nodiscard]] float maxMotorForce() const { return max_motor_force_; }
    [[nodiscard]] float maxSpeed() const { return max_speed_; }
    [[nodiscard]] float steeringAngle() const { return steering_angle_; }
    [[nodiscard]] float maxSteeringAngle() const { return max_steering_angle_; }
    [[nodiscard]] std::array<std::uint8_t, 3> colorBGR() const override
    {
        return color_bgr_;
    }
    [[nodiscard]] Mode mode() const noexcept { return mode_; }
    [[nodiscard]] float brakeForce() const noexcept { return brake_force_; }
    // Setters
    // control variables
    void setSpeed(float speed) const {
        speed = std::clamp(speed, -max_speed_, max_speed_);
        const b2Vec2 vel_relative = { speed * std::cos(steering_angle_),
                             speed * std::sin(steering_angle_) };
        const b2Vec2 vel_world = b2Body_GetWorldVector(body_descriptor_.bodyId, vel_relative);
        b2Body_SetLinearVelocity(body_descriptor_.bodyId, vel_world);
    }

    /**
     * @brief Directly command a motor force. Internally clamped to max_motor_force_.
     * @param force Target motor force in Newtons.
     */
    void setMotorForce(float force);

    /**
     * @brief Set the steering angle of the vehicle, clamped to the maximum allowed.
     * @param angle Desired steering angle in radians.
     */
    void setSteeringAngle(float angle) {
        steering_angle_ = std::clamp(angle, -max_steering_angle_, max_steering_angle_);
    }

    /**
     * @brief Set the color of the vehicle for visualization.
     * @param color_bgr BGR color array.
     */
    void setColor(const std::array<uint8_t,3> color_bgr) { color_bgr_ = color_bgr; }

    /**
     * @brief Set the mode to Rest. The velocity is set to zero immediately, and
     * the steering angle and motor force are also set to zero.
     */
    void rest() {
        logger().info("Vehicle entering rest mode.");
        mode_ = Mode::Rest;
    }

    /**
     * @brief Enter drive mode (allows path following in update()). If
     * no viable path is set, the vehicle will go to brake mode.
     */
    void drive() {
        logger().info("Vehicle entering drive mode.");
        mode_ = Mode::Drive;
    }

    /**
     * @brief Engage braking mode. If force is NaN, defaults to negative current motor force.
     * Ensures stored braking force is always negative and clamped to max magnitude.
     *
     * @note if the vehicle's speed is already very low (<0.1 m/s), it will immediately
     * switch to Rest mode to avoid reversing.
     *
     * @param force Optional braking force (negative). If positive, it will be negated.
     */
    void brake(float force = std::numeric_limits<float>::quiet_NaN());

    /**
     * @brief Get the cell type of this object (Vehicle).
     * @return Cell::Vehicle
     */
    [[nodiscard]] Cell cellType() const noexcept override { return Cell::Vehicle; }

    /** Return the currently attached map or nullptr if detached. */
    [[nodiscard]] Map2D* map() const noexcept { return map_; }; // raw ptr is sufficient because the Map2D class owns its own lifetime

    // Relative position of a sensor on the car body
    enum class MountSide { Front, Back, Left, Right };

    /**
     * @brief Updates the vehicle by continuously applying the current motor force and
     * steering angle.
     *
     * The update depends on the vehicle's current mode:
     * - Drive: Follows the global path using a PID controller for steering and speed.
     *          If no path is set, switches to Brake mode.
     * - Rest:  Immediately stops the vehicle and sets steering and motor force to zero.
     * - Brake: Applies the stored braking force (always negative) to decelerate.
     */
    void update() override;

    /**
     * @brief Add a sensor to the vehicle at a specific mount position.
     * The sensor will be updated with the vehicle's pose when the vehicle moves.
     *
     * @note if the map ptr of the vehicle is not none, the sensor's map ptr will be set
     * to the same map.
     *
     * @param s Pointer to the Sensor object to mount.
     * @param side Side of the vehicle where the sensor is mounted (Front/Back/Left/Right).
     * @param offset Lateral (Left/Right) or longitudinal (Front/Back) shift from the center [m].
     */
    void addSensor(std::unique_ptr<Sensor> s, MountSide side, float offset = 0.0f);

    /**
     * @brief  Register the vehicle as well as its sensors on the map.
     *
     * @note this method will transfer ownership of the sensors to the map. The
     * vehicle then only keeps a non-owning view of the sensors for pose updates.
     *
     * @param map Pointer to the Map2D object where the vehicle is placed.
     */
    void setMap(Map2D *map) override;

    /**
     * @brief Sets the global waypoint for the vehicle to go. Each point
     * in the vector is a node in world coordinates (meters).
     */
    void setGlobalPath(const std::vector<b2Vec2>& path) {
        if (path.size() < 2) {
            logger().warn("Global path should contain at least 2 waypoints (start and target).");
        }
        global_path_ = path;
        if (!global_path_.empty()) {
            mode_ = Mode::Drive; // automatically enter drive mode if a path is provided
        }
    }

    /**
     * @brief Plan a global path from the vehicle's current position to a target location.
     * @param x Target x-coordinate in world meters.
     * @param y Target y-coordinate in world meters.
     */
    void goTo(float x, float y);


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
        Sensor* sensor;
        MountSide side;
        float offset;   ///< lateral (Left/Right) or longitudinal (Front/Back) shift [m]
    };
    std::vector<std::unique_ptr<Sensor>> sensors_;
    std::array<uint8_t,3> color_bgr_;   ///< BGR
    float motor_force_{0.0f};           ///< N
    float max_motor_force_{0.0f};       ///< N, computed as mass * g
    float max_speed_{constants::max_vehicle_speed}; ///< m/s
    float yaw_rate_{0.0f};              ///< rad/s
    float steering_angle_ {0.0f};       ///< rad
    float max_steering_angle_ {M_PI/3};   ///< rad
    std::vector<Mount> mounts_;
    PIDController pid_controller_;          ///< PID controller for path following
    std::vector<b2Vec2> global_path_;       ///< Sequence of waypoints (world coordinates) for global route
    Mode mode_{Mode::Drive};                 ///< Current operational mode
    float brake_force_{-1.0f};             ///< Stored braking force (always negative)
    /**
     * @brief Push updated poses to mounted sensors as the vehicle moves.
     */
    void updateSensors_() const;
    /**
     * @brief Alters the linear damping of the car based on the cell
     * it is currently on. If the cell cannot be found in the friction_map,
     * friction is set to zero.
     */
    void updateFriction_() const;

    /** Compute local path (series of waypoints) ahead of vehicle by traversing
     * the map's graph. The result is every single point that the vehicle will
     * travel to reach the next node in the global path.
     *
     * @note between two nodes, the vehicle travels in a straight line.
     *
     * @note this method is used for local, not global path planning.
     */
    std::vector<b2Vec2> findBestPath();

    /**
     * @brief notifies the box2d world about the current motor force and steering angle
     * applied to the vehicle.
     */
    void notifyBox2DWorld_() const;

};

#endif // VEHICLE_H
