//
// Created by vunha on 8/4/2025.
//

#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "data/classes.h"
#include <../setup/Setup.h>
#include <include/spdlog/spdlog.h>
#include <include/spdlog/sinks/stdout_color_sinks.h>

class Map2D;

/**
 * @brief Describes the physical properties and Box2D configuration for a map object.
 * All objects are assumed to be perfectly rigid with a restitution of 1.0 (no energy loss on collision).
 *
 * This structure encapsulates all relevant Box2D body and shape definitions,
 * including:
 * - body_def: The Box2D body definition (type, position, rotation, velocity, etc.)
 * - shape_def: The Box2D shape definition (density, friction, etc.)
 * - acceleration_vector: The acceleration vector of the body (default: (0,0))
 * - restitution: The restitution coefficient (bounciness, default: 1.0)
 * - bbox: The bounding box polygon shape
 * - bodyId: The Box2D body identifier
 *
 * The constructor initializes the body and its shape in the Box2D world.
 */
struct BodyDescriptor {
    b2BodyDef body_def{b2DefaultBodyDef()};
    b2ShapeDef shape_def{b2DefaultShapeDef()};
    b2Vec2 acceleration_vector{0.0f, 0.0f};
    b2Polygon bbox{}; ///< Bounding box shape
    b2BodyId bodyId{}; ///< Box2D body identifier
    b2ShapeId shapeId{}; ///< Box2D shape identifier

    BodyDescriptor(b2BodyType body_type,
                float rotation,
                float length,
                float width,
                float init_velocity,
                float init_x,
                float init_y,
                float density,
                float friction) {
        if (!b2World_IsValid(WORLD)) {
            throw std::runtime_error("Box2D world is not initialized.");
        }
        body_def.type = body_type;
        body_def.position = {init_x, init_y};
        body_def.rotation = b2MakeRot(rotation);
        initVelocity(init_velocity, rotation);
        shape_def.density = density;
        shape_def.material.friction = friction;
        shape_def.material.restitution = 1.0f; // perfectly elastic
        const float hx = 0.5f * length;
        const float hy = 0.5f * width;
        bbox = b2MakeBox(hx, hy);
        bodyId = b2CreateBody(WORLD, &body_def);
        shapeId = b2CreatePolygonShape(bodyId, &shape_def, &bbox);
        b2Body_ApplyMassFromShapes(bodyId);
        b2Body_SetAngularDamping(bodyId, constants::angular_damping);
    }
    /* Initialize the linear velocity based on speed and angle. */
    void initVelocity(float speed, float angle_rad) {
        body_def.linearVelocity = { speed * std::cos(angle_rad),
            speed * std::sin(angle_rad) };
    }
};

inline std::shared_ptr<spdlog::logger> rootLogger = spdlog::stdout_color_mt("root");

/*
 * @brief Base class for all objects that can be placed on the map. All objects that can move
 * on the map has to inherit from this class.
 */
class MapObject {

public:
    /* Pointer alias for direct cell access. */
    using CellPtr = Cell*;

    /** constructor. Upon creation, registers this object on the map */
    MapObject(float length, float width, b2BodyType body_type = b2_dynamicBody,
              float rotation = 0.0f, float init_velocity = 0.0f, float init_x=0.0f,
              float init_y=0.0f, float density = 1.0f, float friction = 0.0f)
        : body_descriptor_(body_type, rotation, length, width, init_velocity,init_x, init_y, density, friction),
        length_(length),
        width_(width)
    {
    }

    // Non-copyable (identity should be unique).
    MapObject(const MapObject&) = delete;
    MapObject& operator=(const MapObject&) = delete;

    virtual ~MapObject() = default;

    // getters
    [[nodiscard]] b2Vec2 position() const noexcept {
        return b2Body_GetPosition(body_descriptor_.bodyId);
    }
    [[nodiscard]] b2Rot rotation() const noexcept {
        return b2Body_GetRotation(body_descriptor_.bodyId);
    }
    [[nodiscard]] float mass() const { return b2Body_GetMass(body_descriptor_.bodyId); }
    [[nodiscard]] float length() const noexcept { return length_; }
    [[nodiscard]] float width() const noexcept { return width_; }
    [[nodiscard]] bool isStarted() const noexcept { return started_; }
    [[nodiscard]] b2AABB bbox() const noexcept { return b2Body_ComputeAABB(body_descriptor_.bodyId); }
    [[nodiscard]] const b2BodyDef& initialBodyDef() const noexcept { return body_descriptor_.body_def; }
    /**
     * @brief Allows the object to be started updating its position continuously.
     */
    void start() noexcept { started_ = true; }

    /**
     * @brief Stops the object from updating its position.
     */
    void stop() noexcept { started_ = false; }

    /**
     * @brief Sets the linear velocity of the object to zero immediately.
     */
    void freeze() const noexcept {
        b2Body_SetLinearVelocity(body_descriptor_.bodyId, {0.0f, 0.0f});
        b2Body_SetAngularVelocity(body_descriptor_.bodyId, 0.0f);
    }

    /* The cell type associated with this object. */
    [[nodiscard]] virtual Cell cellType() const noexcept = 0;

    /** Update the object's current state for the current time step.
    */
    virtual void update() = 0;

    /** Attach this object to a map so it can read/modify it.
     *
     * @param map: the map to attach the object to
     */
    virtual void setMap(Map2D* map) { this->map_ = map; }

    /** @brief Update the object's world pose.
     *
     * @param x     New x position in meters
     * @param y     New y position in meters
     * @param theta New orientation in radians
     */
    void setPose(float x, float y, float theta) const {
        b2Body_SetTransform(body_descriptor_.bodyId, {x, y}, b2MakeRot(theta));
    }

    /** @brief Update the object's world pose.
     *
     * @param pos   New position vector in meters
     * @param theta New orientation in radians
     */
    void setPose(const b2Vec2& pos, float theta) const {
        b2Body_SetTransform(body_descriptor_.bodyId, pos, b2MakeRot(theta));
    }

    /** @brief Print debug information about the object's physical properties.
     */
    void printPhysicalProperties() const;

    /**
     * @brief BGR color used when rendering dynamic overlays.
     * @return {B, G, R}. Default is a neutral gray; derived types may override.
     */
    [[nodiscard]] virtual std::array<std::uint8_t, 3> colorBGR() const;

    /**
     * @brief Get the world-space corners of this object's axis-aligned local box.
     * @details The corners are returned in clockwise order using the current
     *          Box2D transform and object length/width (meters).
     * @return { (x,y) * 4 } in world meters.
     */
    [[nodiscard]] std::array<b2Vec2, 4> worldBoxCorners() const noexcept;
protected:
    /// **Every derived class overrides this and returns its own logger.**
    [[nodiscard]] virtual spdlog::logger& logger() const = 0;
    BodyDescriptor body_descriptor_; ///< Physics properties including shape, mass, velocity, etc.
    bool started_{false}; ///< Whether the object is started, allowing it to move
    float length_; ///< Length of the object in meters
    float width_;  ///< Width of the object in meters
    Map2D* map_{nullptr}; ///< Pointer to the map this object is attached to
};


#endif //MAPOBJECT_H
