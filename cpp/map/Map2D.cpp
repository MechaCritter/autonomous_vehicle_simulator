#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "../include/nlohmann/json.hpp"
#include "Map2D.h"
#include "../utils/utils.h"
#include "spdlog/sinks/stdout_color_sinks-inl.h"

using json = nlohmann::json;
namespace fs = std::filesystem;
// resources. #TODO:
fs::path cell_color_data_json = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/cell_colors.json";

std::unordered_map<Cell, std::array<std::uint8_t, 3>> Map2D::cell_colors_;

struct CellColorLoader {
    CellColorLoader() {
        Map2D::loadCellColors(cell_color_data_json);
    }
};

// this class was created only so that the cell colors are loaded
static CellColorLoader init_cell_colors;

const std::unordered_map<Cell, std::array<std::uint8_t, 3>> default_cell_colors_ = {
    {Cell::Unknown, {0, 0, 0}}, // black
    {Cell::Free, {64,64,64}}, // dark gray
    {Cell::Road, {128,128,128}}, // light gray
    {Cell::Vehicle, {255,0,255}}, // pink
    {Cell::Obstacle, {0,255,0}}, // green
    {Cell::Reserved, {255,192,255}} // light pink
};

spdlog::logger& Map2D::logger() {
    static std::shared_ptr<spdlog::logger> logger_ =
        spdlog::stderr_color_mt("Map2D");
    return *logger_;
}

Cell Map2D::atPx(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range(std::format(
        "Coordinates out of bounds. Max x range: {}, max y range: {}", width_, height_));
    };
    return map_data_[idx(x, y)];
}

void Map2D::setPx(int x, int y, Cell value) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range(std::format(
        "Coordinates out of bounds. Max x range: {}, max y range: {}", width_, height_));
    };
    map_data_[idx(x, y)] = value;
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
    Map2D local_map =  Map2D(w_window, h_window, resolution_);

    // Set the cell values to the values of the global map
    for (int y = 0; y < h_window; ++y)
        for (int x = 0; x < w_window; ++x)
            local_map.setPx(x, y, atPx(start_x + x, start_y + y));

    //TODO: rotate the local map to the vehicle's orientation. For that, find a way to sneak the car's
    //TODO Pose2D (See the .proto file) into this method.

    return local_map;
}

Cell Map2D::colorToCell(std::array<unsigned char, 3> color) {
    for (const auto& [cell, cell_color] : cell_colors_) {
        if (color == cell_color) {
            return cell;
        }
    }
    throw std::invalid_argument("Color not found in cell colors map");
}

std::array<std::uint8_t, 3> Map2D::cellToColor(Cell cell) {
    return cell_colors_.at(cell);
}

std::string Map2D::cellToString(Cell cell) {
    switch (cell) {
        case Cell::Unknown: return "Unknown";
        case Cell::Free: return "Free";
        case Cell::Road: return "Road";
        case Cell::Vehicle: return "Vehicle";
        case Cell::Obstacle: return "Obstacle";
        case Cell::Reserved: return "Reserved";
        default: throw std::invalid_argument("Invalid cell type");
    }
}

Cell Map2D::stringToCell(const std::string& cell_name) {
    if (cell_name == "Unknown") return Cell::Unknown;
    if (cell_name == "Free") return Cell::Free;
    if (cell_name == "Road") return Cell::Road;
    if (cell_name == "Vehicle") return Cell::Vehicle;
    if (cell_name == "Obstacle") return Cell::Obstacle;
    if (cell_name == "Reserved") return Cell::Reserved;
    throw std::invalid_argument("Invalid cell name: " + cell_name);
}

std::vector<Cell> Map2D::bmpToCells(const std::vector<std::array<uint8_t, 3> > &bmp_matrix) {
    std::vector<Cell> cells;
    for (int i = 0; i < bmp_matrix.size(); ++i) {
        const auto& color = bmp_matrix[i];
        cells.push_back(colorToCell(color));
    }
    return cells;
}

std::vector<std::array<uint8_t, 3>> Map2D::toBmp() const {
    std::vector<std::array<uint8_t, 3>> bmp_matrix;
    for (const auto& cell : map_data_) {
        bmp_matrix.push_back(cellToColor(cell));
    }
    return bmp_matrix;
}

void Map2D::fillMapWith(Cell cell) {
    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x)
            setPx(x, y, cell);
}

void Map2D::loadCellColors(const std::filesystem::path &path) {
    logger().info("Loading cell colors from file: {}", path.string());
    std::ifstream file(path);
    if (!file) {
        std::string path_str = path.string();
        logger().warn("Could not open cell colors file: {}. Using default colors.", path_str);
        for (const auto& [cell, color] : default_cell_colors_) {
            cell_colors_[cell] = color;
        }
        return;
    }
    json j;
    file >> j;
    for (auto& [cell_name, color_json] : j.items()) {
        Cell cell = stringToCell(cell_name);
        std::array<unsigned char, 3> color = color_json.get<std::array<unsigned char, 3>>();
        auto max = *std::ranges::max_element(color);
        auto min = *std::ranges::min_element(color);
        if (max > 255 || min < 0) {
            throw std::invalid_argument("Color values must be in the range [0, 255]");
        }
        cell_colors_[cell] = color;
    }
}

Map2D::Map2D(int width, int height, double res_m, Cell default_value) {
    if (width > MAX_MAP_WIDTH || height > MAX_MAP_HEIGHT) {
        throw std::out_of_range(std::format(
        "Map size exceeds maximum dimensions. Max width: {}, max height: {}", MAX_MAP_WIDTH, MAX_MAP_HEIGHT));
    }
    width_ = width;
    height_ = height;
    resolution_ = res_m;
    //map_ = load_default_map(DEFAULT_MAP_FILE);
    logger().info("Filling map with default value: {}. Map's size: ", cellToString(default_value), cellToString(default_value));
    map_data_.resize(width * height);
    fillMapWith(default_value);

}

Map2D Map2D::loadMap(const std::string &filename, double res_m) {
    int w, h;
    std::vector<std::array<uint8_t, 3>> bmp = utils::loadBmp(filename, w, h);
    std::vector<Cell> cells = bmpToCells(bmp);
    logger().info("Loaded map from file: {}. Size: {}x{}", filename, w, h);
    Map2D m(w, h, res_m);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            m.setPx(x, y, cells[y * w + x]);
    return m;
}

Map2D& globalMap() {
    static Map2D map = Map2D::loadMap(DEFAULT_MAP_FILE, DEFAULT_MAP_RESOLUTION);
    return map;
}

std::ostream& operator<<(std::ostream& os, Cell cell) {
    os << Map2D::cellToString(cell);
    return os;
}

#ifdef WITH_OPENCV_DEBUG
#include <opencv2/videoio.hpp>

void Map2D::startMotionCapture()
{
    capture_frames_.clear();
    capture_active_ = true;
}

void Map2D::addMotionFrame()
{
    if (!capture_active_) return;
    cv::Mat frame(height_, width_, CV_8UC3);
    const auto bmp = toBmp();
    // TODO: parallelize with OpenMP
    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x) {
            const auto& c = bmp[y * width_ + x];                 // RGB → BGR
            frame.at<cv::Vec3b>(y, x) = { c[2], c[1], c[0] };
        }
    capture_frames_.push_back(std::move(frame));
}

void Map2D::endMotionCapture() {
    if (!capture_active_) return;
    if (capture_frames_.empty()) { capture_active_ = false; return; }
    capture_active_ = false;
}

void Map2D::addMotionFrame(const cv::Mat& annotated)
{
    if (!capture_active_) {
        throw std::runtime_error("Motion capture is not active. Call startMotionCapture() first.");
    };

    capture_frames_.push_back(annotated.clone());
}

void Map2D::flushMotionCapture(const std::string& filename, double fps)
{
    if (capture_frames_.empty()) {
        logger().warn("No frames captured. Skipping video creation.");
        return;
    }

    cv::VideoWriter w(filename,
                      cv::VideoWriter::fourcc('m','p','4','v'),
                      fps,
                      capture_frames_[0].size());

    for (const auto& f : capture_frames_) w.write(f);
    w.release();

    capture_frames_.clear();
}

cv::Mat Map2D::renderToMat() const
{
    cv::Mat frame(height_, width_, CV_8UC3);
    const auto bmp = toBmp();                                  // uses existing util :contentReference[oaicite:0]{index=0}
    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x) {
            const auto& c = bmp[y * width_ + x];
            frame.at<cv::Vec3b>(y, x) = { c[2], c[1], c[0] };  // RGB→BGR
        }
    return frame;
}

#endif









