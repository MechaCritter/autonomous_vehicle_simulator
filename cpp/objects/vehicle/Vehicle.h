#ifndef VEHICLE_H
#define VEHICLE_H

#include "../MapObject.h"
#include "../../utils/LoggingUtils.hpp"
#include "../../data/constants.h"
#include <../objects/vehicle/PIDController.h>
#include <../include/CXXGraph/CXXGraph.hpp>
#include <../utils/GraphUtils.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>
#include <expected>
#include <limits>
#include <box2d/box2d.h>


// forward declarations, otherwise circular dependency with Sensor.h
class Sensor;
class Map2D;

/**
 * @note the vehicle owns its own sensors instead of transferring to the map! so destroying the
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
    // gets magnitude of velocity vector
    [[nodiscard]] float absoluteSpeed() const { return b2Length(velocityVector()); }
    // gets projection of velocity onto forward axis
    [[nodiscard]] float forwardSpeed() const noexcept {
        const b2Vec2 v = velocityVector();
        const b2Vec2 fwd = b2Body_GetWorldVector(body_descriptor_.bodyId, constants::unit_x);
        return b2Dot(v, fwd);
    }
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
    [[nodiscard]] float desiredSpeed() const { return desired_speed_; }
    [[nodiscard]] Mode mode() const noexcept { return mode_; }
    [[nodiscard]] float brakeForce() const noexcept { return brake_force_; }
    // Setters
    /**
     * @brief sets the speed of the vehicle to this speed immediately, clamped to the maximum allowed.
     *
     * @note This is a "hard" set, bypassing the PID controller. Use with caution
     * @param speed
     */
    void setSpeed(float speed) const {
        if (speed < 0.0f) {
            throw std::invalid_argument("Speed must be non-negative.");
        }
        speed = std::clamp(speed, 0.0f, max_speed_);
        const b2Vec2 vel_relative = { speed * std::cos(steering_angle_),
                             speed * std::sin(steering_angle_) };
        const b2Vec2 vel_world = b2Body_GetWorldVector(body_descriptor_.bodyId, vel_relative);
        b2Body_SetLinearVelocity(body_descriptor_.bodyId, vel_world);
    }

    /**
     * @brief Set the desired absolute speed of the vehicle, clamped to the maximum allowed. The
     * PID controller will then work its way to achieve this speed.
     *
     * @param speed Desired speed in m/s (non-negative).
     */
    void setDesiredAbsoulteSpeed(float speed) {
        if (speed < 0.0f) {
            throw std::invalid_argument("Desired speed must be non-negative.");
        }
        desired_speed_ = std::clamp(speed, 0.0f, max_speed_);
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
     * @brief disables the controller. Afterwards, the vehicle will use open-loop
     * control and just follow the current steering angle and motor force.
     */
    void disableController() {
        follow_controller_ = false;
    }

    /**
     * @brief enables the controller. Afterwards, the vehicle will try to follow the
     * local path if found.
     */
    void enableController() {
        follow_controller_ = true;
    }

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
     * - Drive: when controller is enabled, follows the global path using a PID controller for steering and speed.
     *          If no path is set, switches to Brake mode. Else, applies current motor force and steering angle (open-loop).
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
     * @detail the vehicle drives until the current node index reaches the last node and
     * the vehicle is close enough to the last waypoint (within stop_distance_). Then it switches to brake mode.
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
     * @brief starts the vehicle to allow it to be updated.
     *
     * @note if the desired speed is 0.0 upon initializing, it
     * will be set to the max allowed speed.
     */
    void startUpdating() noexcept override {
        if (desired_speed_ == 0.0f) {
            desired_speed_ = max_speed_;
        }
        MapObject::startUpdating();
    }

    /**
     * @brief Sets the global waypoint for the vehicle to go. Each point
     * in the vector is a node in world coordinates (meters).
     */
    void setGlobalPath(const std::vector<b2Vec2>& path) {
        if (path.size() < 2) {
            throw std::invalid_argument("Path must contain at least two waypoints.");
        }

        if (path.size() < 3) {
            logger().warn("Path contains only two waypoints. Consider adding intermediate waypoints "
                          "for better path following.");
        }
        global_path_ = path;
        global_head_idx_ = 0;
        if (!global_path_.empty()) {
            mode_ = Mode::Drive; // automatically enter drive mode if a path is provided
        }
    }


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
    float max_brake_force_{0.0f};       ///< N, computed as 4 * mass * g
    float desired_speed_{0.0f};    ///< m/s
    float max_speed_{constants::max_vehicle_speed}; ///< m/s
    float yaw_rate_{0.0f};              ///< rad/s
    float steering_angle_ {0.0f};       ///< rad
    float max_steering_angle_ {M_PI/3};   ///< rad
    bool follow_controller_{true}; ///< whether to use the PID controller for path following. Otherwise, vehicle will go straight.
    std::vector<Mount> mounts_;
    PIDController pid_controller_;          ///< PID controller for path following
    std::vector<b2Vec2> global_path_;       ///< Sequence of waypoints (world coordinates) for global route
    Mode mode_{Mode::Drive};                 ///< Current operational mode
    float brake_force_{-1.0f};             ///< Stored braking force (always negative)
    float stop_distance_{1.5f};          ///< m, distance away from the last waypoint to consider "arrived"
    /** Index of the next global waypoint to consume (0-based).
     *  Invariant: if global_path_.size() >= 2, then 0 <= global_head_idx_ < global_path_.size()-1.
     *  We always follow segments [global_head_idx_] -> [global_head_idx_+1] and onward.
     */
    std::size_t global_head_idx_{0};

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
    std::expected<std::vector<b2Vec2>, std::string> findBestLocalPath_();

    /**
     * @brief notifies the box2d world about the current motor force and steering angle
     * applied to the vehicle.
     */
    void notifyBox2DWorld_() const;

    /**
     * @brief Contains tunable metrics for the local lattice planner.
     *
     *  @param graph_data: the current graph data used for the next time step
     * @param horizon_m: how far ahead (in meters) to plan locally along the global path.
     * The larger, the more foresight, but also the more branching and hence more compute time.
     * @param samples_along_edge_: how many discrete samples of the OBB to create along
     * each edge. The larger, the more accurate collision checking, but also more compute time. If
     * all samples are clear, the edge is considered clear.
     * @param step_m: longitudinal spacing betwen successive layers in the lattice.
     * @param nodes_per_step: how many lateral candidates per layer.
     * @param lateral_spacing_m: lateral offset (in meters) between adjacent candidats in a
     * layer.
     * @param min_distance_between_nodes_: if two nodes are closer than this threshold,
     * they are considered the same.
     */
    struct LatticeConfig {
        GraphData *graph_data{};
        unsigned int samples_along_edge_{8};
        float horizon_m{20.0f};
        float step_m{5.0f};
        int   nodes_per_step{5};
        float lateral_spacing_m{5.0f};
        float min_distance_between_nodes_{1.0f};
    };

    LatticeConfig lattice_config_{};  // per-build lattice config with a fresh graph

    /**
     * @brief Picks the local planning segment `[start, end]` along the global path.
     *
     * @details The code does following steps:
     * 1. Walk the global path segments and accumulate distance until the next segment would
     * exceed the horizon distance.
     * 2. Interpolate within that segment to get the end point.
     *
     * @return pair `[start, end]` where `start` is the vehicle's current position,
     * and `end` is a point along the global path.
     */
    [[nodiscard]] std::pair<b2Vec2,b2Vec2> computeHorizon_() const;

    /**
     * @brief Builds a lattice graph between start and end points, according to the given config.
     * The graph nodes are stored in `g`, and the last layer of nodes is returned in `lastLayer`.
     *
     * @details THis transforms continuous local motion into a small discrere search problem.
     * @note currently, the edge weights depend solely on distance.
     *
     * @param start Starting point of the lattice (vehicle position).
     * @param end   End point of the lattice (along global path).
     * @param lastLayer Output vector of vertex IDs created in the final layer.
     */
    int buildLattice_(const b2Vec2& start,
                        const b2Vec2& end,
                       std::vector<int>& lastLayer) const;

    /**
     * @brief Does collision check to select candidate edges for the motion planner using
     * OBB sweep of the vehicle.
     *
     * @details Does an occupancy test along the selected edge by sampling `samples_along_edge_`
     * OBBs along the edge. If all samples are collision-free, the edge is considered clear.
     *
     * @param a the start point of the edge
     * @param b the end point of the edge
     * @return true if the edge is collision-free, false otherwise.
     */
    [[nodiscard]] bool edgeCollisionFree_(const b2Vec2& a, const b2Vec2& b) const;

    /**
     * @brief choose which node in the final layer should be aimed for.
     *
     * @detail currently, picks the node whose world position is closest to the
     * end point from the horizon cut.
     * @param startNodeId the id of the starting node, from which the distances to each
     * node in the last layer are computed
     * @param lastLayer the vector of node ids in the last layer of the lattice
     * @return the id of the chosen target node
     */
    [[nodiscard]] int chooseTargetNode_(
        unsigned int startNodeId,
        const std::vector<int>& lastLayer) const;

    /**
     * @brief turn the sequence of node IDs from Dijkstra into world waypoints for the controller.
     * @param path vector of node IDs from Dijkstra
     * @return
     */
    [[nodiscard]] std::vector<b2Vec2> reconstructLocalPath_(
        const std::vector<std::string>& path) const;

    /**
     * @brief Advance global_head_idx_ if B=path[head] is behind us along B→C and
     * we’re near the corridor.
     */
    void advanceGlobalHead_();
};

#endif // VEHICLE_H
