#include <stdexcept>
#include <algorithm>
#include "Map2D.h"
#include "../utils/utils.h"

Map2D::Map2D(int width, int height, double res_m, Cell default_value) {
    if (width > MAX_MAP_WIDTH || height > MAX_MAP_HEIGHT) {
        throw std::out_of_range(std::format(
        "Map size exceeds maximum dimensions. Max width: {}, max height: {}", MAX_MAP_WIDTH, MAX_MAP_HEIGHT));
    }
    width_ = width;
    height_ = height;
    res_m_ = res_m;
    //map_ = load_default_map(DEFAULT_MAP_FILE);
    map_ = utils::loadBmp(DEFAULT_MAP_FILE, width, height);
}

Cell Map2D::atPx(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range(std::format(
        "Coordinates out of bounds. Max x range: {}, max y range: {}", width_, height_));
    };
    return map_[idx(x, y)];
}

void Map2D::setPx(int x, int y, Cell value) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range(std::format(
        "Coordinates out of bounds. Max x range: {}, max y range: {}", width_, height_));
    };
    map_[idx(x, y)] = value;
}

Map2D Map2D::window(int cx, int cy, int radius_px) const {
    if (cx < 0 || cy < 0 || cx > width_ || cy > height_) {
        throw std::invalid_argument("Coordinates has to be a positive number between 0 and the grid size");
    }
    // Cap the width and height to fit in the map size
    const int start_x = std::max(0, cx - radius_px);
    const int start_y = std::max(0, cy - radius_px);
    const int end_x = std::min(width_, cx + radius_px);
    const int end_y = std::min(height_, cy + radius_px);
    const int w_window = end_x - start_x;
    const int h_window = end_y - start_y;
    Map2D local_map =  Map2D(w_window, h_window, res_m_);

    // Set the cell values to the values of the global map
    for (int y = 0; y < h_window; ++y)
        for (int x = 0; x < w_window; ++x)
            local_map.setPx(x, y, atPx(start_x + x, start_y + y));

    //TODO: rotate the local map to the vehicle's orientation. For that, find a way to sneak the car's
    //TODO Pose2D (See the .proto file) into this method.

    return local_map;
}

Map2D Map2D::loadMap(const std::string &filename, double res_m) {
    int w, h;
    std::vector<Cell> cells = utils::loadBmp(filename, w, h);
    Map2D m(w, h, res_m);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            m.setPx(x, y, cells[y * w + x]);
    return m;
}

const Map2D& globalMap() {
    static Map2D map = Map2D::loadMap(DEFAULT_MAP_FILE, DEFAULT_MAP_RESOLUTION);
    return map;
}







