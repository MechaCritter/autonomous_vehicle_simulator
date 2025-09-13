#pragma once
#include <box2d/box2d.h>
#include <cassert>
#include <chrono>
#include <mutex>

inline b2WorldId WORLD;

/**
 * @brief Prevents multiple threads from updating/accessing the Box2D world simultaneously.
 *
 * @note always use this mutex when trying to access the variable `WORLD`!!! Otherwise, weird
 * data races could occur.
 */
inline std::mutex world_mutex;

/**
 * @brief Global simulation clock for consistent timing across threads
 */
inline std::chrono::steady_clock::time_point global_simulation_clock = std::chrono::steady_clock::now();


/**
 * Create the Box2D world once. If the world already exists, do nothing.
 *
 * The world has zero gravity (top-down view).
 */
inline void setupWorld() {
    std::lock_guard lock(world_mutex);
    if (b2World_IsValid(WORLD))  // <-- use validation API
        return;

    b2WorldDef wdef = b2DefaultWorldDef();
    wdef.gravity = (b2Vec2){0.0f, 0.0f};
    WORLD = b2CreateWorld(&wdef);
    assert(b2World_IsValid(WORLD) && "b2CreateWorld failed");
}

/**
 * @brief Casts a ray in the world using Box2D's ray-casting functionality.
 */
inline void castRayWorld(const b2Vec2& origin, const b2Vec2& translation,
                         const b2QueryFilter& filter,
                         b2CastResultFcn callback, void* ctx) {
    std::lock_guard lock(world_mutex);
    if (!b2World_IsValid(WORLD))
        throw std::runtime_error("Box2D world is not initialized.");
    (void)b2World_CastRay(WORLD, origin, translation, filter, callback, ctx);
}


/**
 * @brief update the current physics world.
 */
inline void updateWorld(std::chrono::steady_clock::time_point current_time) {
    std::lock_guard lock(world_mutex);
    if (!b2World_IsValid(WORLD))
        throw std::runtime_error("Box2D world is not initialized.");
    auto time_diff = std::chrono::duration<float>(current_time - global_simulation_clock).count();
    b2World_Step(WORLD, time_diff, 4);
    global_simulation_clock = current_time;
}

/**
 * Destroy the world if it exists.
 */
inline void destroyWorld()
{
    std::lock_guard lock(world_mutex);
    if (!b2World_IsValid(WORLD))
        return;

    b2DestroyWorld(WORLD);
    WORLD = {}; // zero -> null id
}
