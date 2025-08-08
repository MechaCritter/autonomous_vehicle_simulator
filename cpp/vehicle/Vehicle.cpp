#include "Vehicle.h"
#include "../sensors/Sensor.h"
#include "../config/constants.h"
#include <eigen3/Eigen/Dense>
#include <spdlog/spdlog.h>

Vehicle::Vehicle(Pose2D init_pose,
                 double length, double width,
                 std::array<uint8_t,3> color_bgr,
                 double init_speed,
                 double init_accel)
    : MapObject(init_pose, length, width),
      color_bgr_(color_bgr),
      speed_(init_speed),
      accel_(init_accel)
{
    /* Compute inertial properties -------------------------------------- */
    mass_  = constants::vehicle_density * length * width;
    max_motor_force_ = mass_ * constants::g;              // traction-limited
    motor_force_     = mass_ * accel_;
}

void Vehicle::addSensor(Sensor* s, MountSide side, double offset)
{
    mounts_.push_back({s, side, offset});
    // Add sensor mass to vehicle's total mass
    mass_ += s->mass();
    updateSensors();   // place immediately
}

void Vehicle::setMap(Map2D *map) {
    MapObject::setMap(map);
    for (auto& m : mounts_) {
        map->addObject(*m.sensor); // register sensor on the map
    }
    updateSensors(); // place sensors immediately
}


void Vehicle::updateSensors() const {
    for (auto& m : mounts_) {
        Eigen::Vector2d rel;          // body-frame offset
        switch (m.side) {
            case MountSide::Front: rel = { length_/2.0 + m.offset, 0.0 }; break;
            case MountSide::Back:  rel = {-length_/2.0 - m.offset, 0.0 }; break;
            case MountSide::Left:  rel = { 0.0,  width_/2.0 + m.offset }; break;
            case MountSide::Right: rel = { 0.0, -width_/2.0 - m.offset }; break;
        }
        // rotate into world frame
        double c = std::cos(pose_.theta), s = std::sin(pose_.theta);
        Eigen::Vector2d world = { pose_.x + c*rel.x() - s*rel.y(),
                                  pose_.y + s*rel.x() + c*rel.y() };
        Pose2D p{ world.x(), world.y(), pose_.theta };
        m.sensor->setPose(p);
        m.sensor->update();
    }
}

void Vehicle::update() {
    updating_ = true;
    speed_ = std::clamp(speed_ + accel_ * constants::step_size, 0.0, max_speed_);
    yaw_rate_ = std::tan(steering_angle_) / length_ * speed_;
    const double theta0 = pose_.theta; // save current pose

    /* --- after pose update, move all mounted sensors ------------------- */
    if (std::fabs(yaw_rate_) < 1e-6) {
        // Going straight (no rotation)
        pose_.x += speed_ * std::cos(theta0) * constants::step_size;
        pose_.y += speed_ * std::sin(theta0) * constants::step_size;
        // theta remains the same
    } else {
        // Moving along an arc of constant yaw_rate
        const double theta1 = theta0 + yaw_rate_ * constants::step_size;
        const double R = speed_ / yaw_rate_;  // turning radius
        pose_.x += R * (std::sin(theta1) - std::sin(theta0));
        pose_.y += R * (-std::cos(theta1) + std::cos(theta0));
        pose_.theta = std::fmod(theta1, 2.0 * M_PI); // modulo to avoid theta growing too large
    }
    /* recompute pixel footprint for next Map2D::update() */
    updateFootprint();
    /* --- after pose update, move all mounted sensors ------------------- */
    updateSensors();
}

void Vehicle::setAcceleration(double accel)
{
    const double req_F = mass_ * accel;
    if (std::fabs(req_F) > max_motor_force_) {
        logger().warn("Requested acceleration {:.2f} m/s² requires {:.1f} N > {:.1f} N. Clamping.",
                      accel, req_F, max_motor_force_);
        motor_force_ = std::copysign(max_motor_force_, req_F);
        accel_ = motor_force_ / mass_;
    } else {
        accel_ = accel;
        motor_force_ = req_F;
    }
}

void Vehicle::setMotorForce(double force)
{
    if (std::fabs(force) > max_motor_force_) {
        logger().warn("Requested motor force {:.1f} N exceeds max {:.1f} N. Clamping.",
                      force, max_motor_force_);
        motor_force_ = std::copysign(max_motor_force_, force);
    } else {
        motor_force_ = force;
    }
    accel_ = motor_force_ / mass_;
}
