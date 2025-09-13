#include "../vehicle/Vehicle.h"
#include "../sensors/Sensor.h"
#include "../../data/constants.h"
#include <spdlog/spdlog.h>
#include <limits>
#include <cmath>

Vehicle::Vehicle(float length, float width,
                 std::array<uint8_t,3> color_bgr,
                 float rotation,
                 float init_speed,
                 float init_x,
                 float init_y,
                 float motor_force)
    : MapObject(length, width, b2_dynamicBody, rotation, init_speed, init_x, init_y,
        constants::vehicle_density, constants::vehicle_road_friction),
      color_bgr_(color_bgr)
{
    drive();
    max_motor_force_ = std::abs(mass() * constants::g);    // traction-limited
    pid_controller_.setSpeedLimits({-max_motor_force_, max_motor_force_, -50.0f, 50.0f});
    pid_controller_.setSteerLimits({-max_steering_angle_, max_steering_angle_, -0.2f, 0.2f});
    setMotorForce(motor_force); // initial motor force
}

Vehicle::~Vehicle() = default;

void Vehicle::addSensor(std::unique_ptr<Sensor> s, MountSide side, float offset)
{
    Sensor* sensor_ptr = s.get();
    sensors_.push_back(std::move(s)); // transfers ownership to vehicle
    if (map_) {
        setMap(map_); // re-register sensors on the map
    }
    // Provide owner/body/filter so the sensor can ignore us and reuse our collidable set
    sensor_ptr->setOwner(this);
    mounts_.push_back({sensor_ptr, side, offset}); // non-owning view used for pose updates
    updateSensors_();   // place immediately
}

void Vehicle::setMap(Map2D *map) {
    MapObject::setMap(map);
    for (auto& sensor : sensors_) {
        sensor->setMap(map);
    }
    updateSensors_(); // place sensors immediately
}

void Vehicle::updateSensors_() const {
    const float halfL = 0.5f * length();
    const float halfW = 0.5f * width();
    const b2BodyId id = body_descriptor_.bodyId;
    const b2Rot rot = rotation();
    const float theta = b2Rot_GetAngle(rot);

    for (auto& m : mounts_) {
        // Local (body-frame) mount offset
        b2Vec2 rel;
        switch (m.side) {
            case MountSide::Front:
                rel = { halfL + m.offset, 0.0f };
                break;
            case MountSide::Back:
                rel = { -halfL - m.offset, 0.0f };
                break;
            case MountSide::Left:
                rel = { 0.0f, halfW + m.offset};
                break;
            case MountSide::Right:
                rel = { 0.0f, -halfW - m.offset};
                break;
            default: // fallback to Front
                logger().warn("Unknown mount side. Defaulting to Front.");
                rel = { halfL + m.offset, 0.0f };
                break;
        }
        // Convert to world using Box2D helper
        const b2Vec2 worldP = b2Body_GetWorldPoint(id, rel);
        m.sensor->setPose(worldP, theta);
    }
}

void Vehicle::update() {
    // Calculate dt from this vehicle's last update time
    auto current_time = std::chrono::steady_clock::now();

    // Initialize last_update_time_ on first call
    if (last_updated_.time_since_epoch().count() == 0) {
        last_updated_ = current_time;
    }
    
    float dt = std::chrono::duration<float>(current_time - last_updated_).count();
    last_updated_ = current_time;
    
    // Use a minimum dt to avoid division by zero or very small values
    dt = std::max(dt, 1e-6f);
    while (true) {
        switch (mode_) {
            case Mode::Drive: {
                if (!global_path_.empty()) {
                    // Perform local planning and control if a target path is set
                    std::vector<b2Vec2> local_path = findBestPath();
                    if (local_path.empty()) {
                        // No viable local path: stop the vehicle (fall through to braking style decel)
                        setSteeringAngle(0.0f);
                        setMotorForce(0.0f);
                    } else {
                        // Choose a point ahead to track (e.g., the first waypoint on the local path)
                        b2Vec2 target = local_path.size() > 1 ? local_path[1] : local_path[0];
                        // Compute control outputs
                        float current_speed = speed();
                        float desired_speed = 5.0f;  // desired cruising speed (m/s)
                        float current_angle = b2Rot_GetAngle(rotation());
                        auto [steer_ang, motor_force] = pid_controller_.compute(current_angle, position(), target, current_speed, desired_speed, dt);
                        logger().debug("PID outputs: steer angle {:.3f} rad, motor force {:.1f} N", steer_ang, motor_force);
                        // Apply steering and throttle commands (with safety clamping)
                        setSteeringAngle(steer_ang);
                        setMotorForce(motor_force);
                    }
                    break;
                }
                // No path -> hold still
                setSteeringAngle(0.0f);
                setMotorForce(0.0f);
                mode_ = Mode::Brake; // switch to brake mode
                logger().warn("No global path set. Switching to Brake mode.");
                continue;
            }
            case Mode::Rest: {
                setSteeringAngle(0.0f);
                setMotorForce(0.0f);
                freeze();
                break;
            }
            case Mode::Brake: {
                if (speed() < 0.1f) {
                    // Vehicle is nearly stopped: switch to Rest mode to avoid reversing
                    mode_ = Mode::Rest;
                    logger().debug("Vehicle speed is low: {:.2f} m/s. Switching to Rest mode.", speed());
                    continue; // re-evaluate in Rest mode
                }
                setSteeringAngle(0.0f); // keep wheels straight during braking
                setMotorForce(brake_force_);
                break;
            }
            default: {
                throw std::runtime_error("Unknown vehicle mode.");
            }
        }
        break;
    }
    notifyBox2DWorld_();
}

void Vehicle::updateFriction_() const {
    if (map_) {
        const b2BodyId id = body_descriptor_.bodyId;
        const b2Vec2 pos = b2Body_GetPosition(id);
        const auto [px, py] = map_->worldToCell(pos.x, pos.y);
        const Cell ground = map_->atPx(px, py);
        auto search = friction_map.find(ground);
        float linDamp = search != friction_map.end() ? search->second : 0.0f;
        b2Body_SetLinearDamping(id,  linDamp);
    }
}


void Vehicle::setMotorForce(float force)
{
    if (std::fabs(force) > max_motor_force_) {
        logger().warn("Requested motor force {:.1f} N exceeds max {:.1f} N. Clamping.",
                      force, max_motor_force_);
        motor_force_ = std::copysign(max_motor_force_, force);
    } else {
        motor_force_ = force;
    }
}

void Vehicle::brake(float force) {
    logger().info("Vehicle entering brake mode with force {:.1f} N.", force);
    if (std::isnan(force)) {
        force = -motor_force_; // default: negative of current motor force
    }
    force = force < 0.0f ? force : -force; // ensure negative

    brake_force_ = force;
    mode_ = Mode::Brake;
}

std::vector<b2Vec2> Vehicle::findBestPath() {
    // todo: implement!
    return global_path_;
}

void Vehicle::notifyBox2DWorld_() const {
    const b2BodyId id = body_descriptor_.bodyId;
    // --- 1) Apply drive force at the steered FRONT axle (off-COM) ---
    const b2Vec2 frontLocal = {+0.5f * length(), 0.0f};
    const b2Vec2 wheelDirLocal = {std::cos(steering_angle_),
                                std::sin(steering_angle_)}; // +X rotated by steering

    // Convert to world
    const b2Vec2 frontWorld = b2Body_GetWorldPoint(id, frontLocal);
    const b2Vec2 wheelDirWorld  = b2Body_GetWorldVector(id, wheelDirLocal);

    // Drive force vector (traction-limited by setMotorForce / setAcceleration)
    const b2Vec2 Fdrive = { motor_force_ * wheelDirWorld.x,
                            motor_force_ * wheelDirWorld.y };
    b2Body_ApplyForce(id, Fdrive, frontWorld, true);
    updateFriction_();
    // "teleport" the sensors to the new vehicle pose
    updateSensors_();
}
