#ifndef VEHICLE_H
#define VEHICLE_H

#include "MapObject.h"
#include "../utils/logging.h"
#include "../data/constants.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>
#include <box2d/box2d.h>


// forward declarations, otherwise circular dependency with Sensor.h
class Sensor;
class Map2D;

class Vehicle final : public MapObject {
public:
    /**
     * @brief Construct a Vehicle with given physical properties and initial state.
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

    // Getters
    [[nodiscard]] std::array<uint8_t,3> color() const { return color_bgr_; }
    [[nodiscard]] b2Vec2 velocityVector() const noexcept {
        return b2Body_GetLinearVelocity(body_descriptor_.bodyId);
    }
    [[nodiscard]] float speed() const { return b2Length(velocityVector()); }

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
    // Setters
    // control variables
    void setSpeed(float speed) {
        speed = std::clamp(speed, -max_speed_, max_speed_);
        const b2Vec2 vel = { speed * std::cos(steering_angle_),
                             speed * std::sin(steering_angle_) };
        b2Body_SetLinearVelocity(body_descriptor_.bodyId, vel);
    }

    /**
     * @brief Directly command a motor force. Internally
     * clamped to maxForceOutput.
     *
     *    @param force Target motor force in Newtons.
     */
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
    void setColor(std::array<uint8_t,3> color_bgr) { color_bgr_ = color_bgr; }

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
     * @brief Updates the vehicle by continuously applying the current motor force
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
    /**
     * @brief Push updated poses to mounted sensors as the vehicle moves.
     *
     */
    void updateSensors_() const;
    /**
     * @brief Alters the linear damping of the car based on the cell
     * it is currently on.
     */
    void updateFriction_() const;
};

#endif // VEHICLE_H
