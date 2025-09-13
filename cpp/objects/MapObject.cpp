#include "../objects/MapObject.h"
#include "../data/DataClasses.h"
#include <cmath>

void MapObject::printPhysicalProperties() const {
    const b2Vec2 pos = position();
    const b2Rot rot = rotation();
    const b2Vec2 velocity = b2Body_GetLinearVelocity(body_descriptor_.bodyId);
    const float angular_velocity = b2Body_GetAngularVelocity(body_descriptor_.bodyId);
    const b2AABB aabb = bbox();
    
    logger().info("=== MapObject Debug Info ===");
    logger().info("Position: ({:.3f}, {:.3f})", pos.x, pos.y);
    logger().info("Rotation: {:.3f} rad ({:.1f} deg)", b2Rot_GetAngle(rot), b2Rot_GetAngle(rot) * 180.0f / M_PI);
    logger().info("Linear Velocity: ({:.3f}, {:.3f})", velocity.x, velocity.y);
    logger().info("Angular Velocity: {:.3f} rad/s", angular_velocity);
    logger().info("Mass: {:.3f} kg", mass());
    logger().info("Body Type: {}", body_descriptor_.body_def.type == b2_staticBody ? "Static" : 
                                   body_descriptor_.body_def.type == b2_kinematicBody ? "Kinematic" : "Dynamic");
    logger().info("Density: {:.3f}", body_descriptor_.shape_def.density);
    logger().info("Friction: {:.3f}", body_descriptor_.shape_def.material.friction);
    logger().info("Restitution: {:.3f}", body_descriptor_.shape_def.material.restitution);
    logger().info("AABB: ({:.3f}, {:.3f}) to ({:.3f}, {:.3f})", 
                  aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x, aabb.upperBound.y);
    logger().info("Started: {}", started_ ? "Yes" : "No");
    logger().info("Cell Type: {}", static_cast<int>(cellType()));
    logger().info("=============================");
}

std::array<std::uint8_t, 3> MapObject::colorBGR() const
{
    // Get the color based on the object's cell type from the global cell_colors map
    return cellToColor(cellType());
}

std::array<b2Vec2, 4> MapObject::worldBoxCorners() const noexcept
{
    const float hx = 0.5f * length_;
    const float hy = 0.5f * width_;

    // local box corners (+x is forward if you defined it so in your shapes)
    const std::array<b2Vec2, 4> local = {
        b2Vec2{ +hx, +hy },
        b2Vec2{ +hx, -hy },
        b2Vec2{ -hx, -hy },
        b2Vec2{ -hx, +hy }
    };

    std::array<b2Vec2, 4> world{};
    for (size_t i = 0; i < 4; ++i) {
        world[i] = b2Body_GetWorldPoint(body_descriptor_.bodyId, local[i]);
    }
    return world;
}

