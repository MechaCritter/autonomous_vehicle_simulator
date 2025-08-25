#pragma once
#include <box2d/box2d.h>
#include <../data/constants.h>
#include <cassert>

inline b2WorldId WORLD;

/**
 * Create the Box2D world once. If the world already exists, do nothing.
 */
inline void setupWorld()
{
    if (b2World_IsValid(WORLD))  // <-- use validation API
        return;

    b2WorldDef wdef = b2DefaultWorldDef();
    wdef.gravity = (b2Vec2){0.0f, 0.0f};

    WORLD = b2CreateWorld(&wdef);
    assert(b2World_IsValid(WORLD) && "b2CreateWorld failed");
}

/**
 * Destroy the world if it exists.
 */
inline void destroyWorld()
{
    if (!b2World_IsValid(WORLD))
        return;

    b2DestroyWorld(WORLD);
    WORLD = {}; // zero -> null id
}
