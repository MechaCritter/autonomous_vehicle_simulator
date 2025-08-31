## Some important notes

### The coordinate system

The origin of the `world coordinate system` is at the **top left corner** of the
map. The `x` axis points to the right and the `y` axis points down (just like how 
OpenCV's image coordinate system works).

### The Box2d World

***IMPORTANT***: The variable "b2WorldID WORLD" (under setup/Setup.h) represents the Box2D world. Wherever this
variable is used with a Box2D function, always use a lock guard combined with the mutex `world_mutex`!!! Otherwise,
weird data races could occur.