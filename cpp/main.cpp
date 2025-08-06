#include <iostream>
#include <string>
#include <spdlog/spdlog.h>
#include "sensors/Lidar2D.h"
#include "map/Map2D.h"
#include "utils/utils.h"
#include "vehicle/Vehicle.h"
#include <vector>

int main() {
    Pose2D startPose{50.0, 50.0, 0.0}; // Starting position of the vehicle
    double dt = 0.1; // Time step for simulation
    Lidar2D lidar("lidar", "front", nullptr, 10, startPose); // Create a Lidar sensor
    auto map = globalMap();
    Vehicle car(startPose, 4.0, 2.0, {0,0,255});
    car.addSensor(&lidar, Vehicle::MountSide::Front);
    car.update(dt);                // integrates accel → speed → pose
    map.stampVehicle(car);         // mark occupancy before rendering
}

