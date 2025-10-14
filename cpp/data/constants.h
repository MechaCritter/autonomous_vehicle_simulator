/*
 * @brief Defines globally used constants.
 *
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <box2d/box2d.h>

namespace constants {
    // FPS for map updates
    inline constexpr int FPS = 30; // Frames / second
    inline constexpr float step_size = 1.0f / FPS; // seconds
    inline constexpr int max_offmap_time = 10; // 60 secs
    inline constexpr float vehicle_density = 200.0f ; // kg/m^2
    inline constexpr float max_vehicle_speed = 60.0f; // m/s
    inline constexpr float g = -9.81f; // m/s^2
    inline constexpr float restitution = 0.3f;   ///< 0=no bounce, 1=perfectly elastic
    inline constexpr float linear_damping = 0.2f;  ///< simulates friction / drag
    inline constexpr float angular_damping = 0.2f; ///< simulates rotational friction / drag
    inline constexpr float vehicle_road_friction = 1.0f;
    inline constexpr b2Vec2 unit_x = {1.0f, 0.0f};
    inline constexpr b2Vec2 unit_y = {0.0f, 1.0f};
}

#endif //CONSTANTS_H
