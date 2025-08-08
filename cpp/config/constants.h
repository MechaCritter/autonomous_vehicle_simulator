/*
 * @brief Defines globally used constants.
 *
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace constants {
    // FPS for map updates
    inline constexpr int FPS = 30; // Frames / second
    inline constexpr double step_size = 1.0 / FPS; // seconds
    inline constexpr double vehicle_density = 10.0 ; // kg/m^2
    inline constexpr double max_vehicle_speed = 60.0; // m/s
    inline constexpr double sensors_density = 1.0; // kg/m^2
    inline constexpr float g = 9.81f; // m/s^2
}

#endif //CONSTANTS_H
