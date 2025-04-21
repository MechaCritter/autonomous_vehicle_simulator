#include <iostream>
#include <string>
#include <spdlog/spdlog.h>
#include "sensors/Lidar2D.h"
#include "map/Map2D.h"
#include "utils/utils.h"
#include <vector>

int main() {
    Lidar2D lidar("Lidar1", "front");
    std::cout << "Sensor name: " << lidar.name() << std::endl;

    std::vector input = {1, 2, 3, 4, 5};
    std::unique_ptr<sensor_data::SensorData> data = lidar.generateData(input);

    if (data->has_lidar_scan_2d()) {
        float angleMin = data->lidar_scan_2d().angle_min();
        std::cout << "Angle min: " << angleMin << std::endl;
    } else {
        std::cout << "No LidarScan2D data available." << std::endl;
    }

    const Map2D map = globalMap();
    std::cout << "Map width: " << map.width() << std::endl;
    std::cout << "Map height: " << map.height() << std::endl;
    std::cout << "Map resolution: " << map.resolution() << std::endl;
    std::cout << "Map cell at (0, 0): " << static_cast<int>(map.atPx(0, 0)) << std::endl;
    std::cout << "Map cell class at (0, 0): " << map.atPx(0, 0) << std::endl;
}

