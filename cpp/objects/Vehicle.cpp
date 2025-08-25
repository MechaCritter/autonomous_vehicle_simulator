#include "../objects/Vehicle.h"
#include "../objects/sensors/Sensor.h"
#include "../data/constants.h"
#include <spdlog/spdlog.h>

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
    max_motor_force_ = std::abs(mass() * constants::g);    // traction-limited
    setMotorForce(motor_force); // initial motor force
}

void Vehicle::addSensor(Sensor* s, MountSide side, float offset)
{
    mounts_.push_back({s, side, offset});
    // // Add sensor mass to vehicle's total mass
    // body_descriptor_.mass += s->mass();
    updateSensors_();   // place immediately
}

void Vehicle::setMap(Map2D *map) {
    MapObject::setMap(map);
    for (auto& m : mounts_) {
        map->addObject(*m.sensor); // register sensor on the map
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
            default: // fallback to Front
                logger().warn("Unknown mount side. Defaulting to Front.");
                rel = { halfL + m.offset, 0.0f };
                break;
            case MountSide::Back:
                rel = { -halfL - m.offset, 0.0f };
                break;
            case MountSide::Left:
                rel = { 0.0f, halfW + m.offset };
                break;
            case MountSide::Right:
                rel = { 0.0f, -halfW - m.offset };
                break;
        }
        // Convert to world using Box2D helper
        const b2Vec2 worldP = b2Body_GetWorldPoint(id, rel);
        m.sensor->setPose(worldP, theta);
    }
}

void Vehicle::update() {
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

void Vehicle::updateFriction_() const {
    if (map_) {
        const b2BodyId id = body_descriptor_.bodyId;
        const b2Vec2 pos = b2Body_GetPosition(id);
        const auto [px, py] = map_->worldToCell(pos.x, pos.y);
        const Cell ground = map_->atPx(px, py);
        auto linDamp = friction_map[ground];
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

