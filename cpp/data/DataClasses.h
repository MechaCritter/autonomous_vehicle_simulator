#ifndef CLASSES_H
#define CLASSES_H
#include <variant>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <array>
#include <string>
#include <filesystem>
#include <box2d/box2d.h>
#include "../include/nlohmann/json.hpp"


using json = nlohmann::json;

// Forward declarations for sensor data types
struct LidarScan2D;
struct Odometry2D;
struct Imu2D;
struct CameraFrame2D;

// Sensor data types
using SensorDataVariant = std::variant<
    LidarScan2D,
    Odometry2D,
    Imu2D,
    CameraFrame2D>;

/**
 * @brief Represents different types of cells in a grid-based environment.
 * This will be used for the `Map2D` class to represent the state of each cell in the map.
 */
enum class Cell : std::uint8_t {
    Unknown = 0,
    Free = 1,
    Road = 2,
    Vehicle = 3,
    Lidar = 4,
    Obstacle = 5,
    Reserved = 6,
    Grass = 7
};

inline const std::unordered_map<Cell, float> friction_map = {
    {Cell::Free, 0.0f},      // Free space
    {Cell::Road, 0.9f},      // Road surface
    {Cell::Vehicle, 1.0f},   // Vehicle on vehicle
    {Cell::Lidar, 0.0f},     // Lidar sensor area
    {Cell::Obstacle, 0.9f},  // Obstacle area
    {Cell::Reserved, 0.00001f},   // Reserved area
    {Cell::Grass, 0.5f}     // Grass area
};

// Cell colors map - maps each cell type to its corresponding BGR color
inline std::unordered_map<Cell, std::array<std::uint8_t, 3>> cell_colors = {
    {Cell::Unknown, {0, 0, 0}},        // black
    {Cell::Free, {64, 64, 64}},        // dark gray
    {Cell::Road, {128, 128, 128}},     // light gray
    {Cell::Vehicle, {255, 0, 255}},    // pink
    {Cell::Lidar, {0, 64, 128}},       // dark blue
    {Cell::Obstacle, {82, 81, 69}},     // dark gray
    {Cell::Reserved, {255, 192, 255}}  // light pink
};

// Cell utility functions
/**
 * @brief Converts a BGR color array to a corresponding Cell type.
 * if the color does not match any known cell type, returns Cell::Unknown.
 */
inline Cell colorToCell(const std::array<std::uint8_t, 3>& color) {
    for (const auto& [cell, cell_color] : cell_colors) {
        if (color == cell_color) {
            return cell;
        }
    }
    return Cell::Unknown;
}

/**
 * @brief Converts a Cell type to its corresponding BGR color
 */
inline std::array<std::uint8_t, 3> cellToColor(Cell cell) {
    return cell_colors.at(cell);
}

/**
 * @brief Converts a Cell type to a string representation
 */
inline std::string cellToString(Cell cell) {
    switch (cell) {
        case Cell::Unknown: return "Unknown";
        case Cell::Free: return "Free";
        case Cell::Road: return "Road";
        case Cell::Vehicle: return "Vehicle";
        case Cell::Lidar: return "Lidar";
        case Cell::Obstacle: return "Obstacle";
        case Cell::Reserved: return "Reserved";
        case Cell::Grass: return "Grass";
        default: throw std::invalid_argument("Invalid cell type");
    }
}

/**
 * @brief Converts the string representation of a cell type to the corresponding Cell enum value
 */
inline Cell stringToCell(const std::string& cell_name) {
    if (cell_name == "Unknown") return Cell::Unknown;
    if (cell_name == "Free") return Cell::Free;
    if (cell_name == "Road") return Cell::Road;
    if (cell_name == "Vehicle") return Cell::Vehicle;
    if (cell_name == "Lidar") return Cell::Lidar;
    if (cell_name == "Obstacle") return Cell::Obstacle;
    if (cell_name == "Reserved") return Cell::Reserved;
    if (cell_name == "Grass") return Cell::Grass;
    throw std::invalid_argument("Invalid cell name: " + cell_name);
}

/**
 * @brief Loads the cells colors from a json file
 */
inline void loadCellColors(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Warning: Could not open cell colors file: " << path.string()
                  << ". Using default colors." << std::endl;
        return; // Keep default colors that are already initialized
    }

    try {
        json j;
        file >> j;
        for (auto& [cell_name, color_json] : j.items()) {
            Cell cell = stringToCell(cell_name);
            std::array<std::uint8_t, 3> color = color_json.get<std::array<std::uint8_t, 3>>();
            cell_colors[cell] = color;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading cell colors: " << e.what()
                  << ". Using default colors." << std::endl;
    }
}

#endif //CLASSES_H
