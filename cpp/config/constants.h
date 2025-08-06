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
}

#endif //CONSTANTS_H
