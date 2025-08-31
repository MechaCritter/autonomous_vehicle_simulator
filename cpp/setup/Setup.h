#pragma once
#include <box2d/box2d.h>
#include <cassert>

inline b2WorldId WORLD;
/**
 * @brief Prevents multiple threads from updating/accessing the Box2D world simultaneously.
 *
 * @note always use this mutex when trying to access the variable `WORLD`!!! Otherwise, weird
 * data races could occur.
 */
inline std::mutex world_mutex;

/**
 * Create the Box2D world once. If the world already exists, do nothing.
 *
 * The world has zero gravity (top-down view).
 */
inline void setupWorld() {
    if (b2World_IsValid(WORLD))  // <-- use validation API
        return;

    b2WorldDef wdef = b2DefaultWorldDef();
    wdef.gravity = (b2Vec2){0.0f, 0.0f};
    {
        std::lock_guard lock(world_mutex);
        WORLD = b2CreateWorld(&wdef);
        assert(b2World_IsValid(WORLD) && "b2CreateWorld failed");
    }
}

/**
 * Destroy the world if it exists.
 */
inline void destroyWorld()
{
    if (!b2World_IsValid(WORLD))
        return;

    {
        std::lock_guard lock(world_mutex);
        b2DestroyWorld(WORLD);
        WORLD = {}; // zero -> null id
    }
}
