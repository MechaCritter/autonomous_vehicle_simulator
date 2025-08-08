#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <opencv2/videoio.hpp>
#include <eigen3/Eigen/Dense>
#include "config/constants.h"
#include "../include/nlohmann/json.hpp"
#include "base/MapObject.h"
#include "map/Map2D.h"

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
    {Cell::Lidar, {0, 64, 128}}, // dark blue
    {Cell::Obstacle, {0,255,0}}, // green
    {Cell::Reserved, {255,192,255}} // light pink
};

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
        case Cell::Lidar: return "Lidar";
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
    if (cell_name == "Lidar") return Cell::Lidar;
    if (cell_name == "Obstacle") return Cell::Obstacle;
    if (cell_name == "Reserved") return Cell::Reserved;
    throw std::invalid_argument("Invalid cell name: " + cell_name);
}

std::vector<Cell> Map2D::bmpToCells(const std::vector<std::array<uint8_t, 3> > &bmp_matrix) {
    std::vector<Cell> cells;
    for (auto color : bmp_matrix) {
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

Map2D::Map2D(int width, int height, double res_m) {
    if (width > MAX_MAP_WIDTH || height > MAX_MAP_HEIGHT) {
        throw std::out_of_range(std::format(
        "Map size exceeds maximum dimensions. Max width: {}, max height: {}", MAX_MAP_WIDTH, MAX_MAP_HEIGHT));
    }
    width_ = width;
    height_ = height;
    resolution_ = res_m;
    //map_ = load_default_map(DEFAULT_MAP_FILE);
    map_data_.resize(width * height);
    base_data_.resize(width * height);
    objects_.clear();
}

Map2D::Map2D(int width, int height, double res_m, Cell default_value)
    :Map2D(width, height, res_m)  // call the other constructor
{
    logger().info("Filling map with default value: {}", cellToString(default_value));
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

void Map2D::removeFootprints() const {
    for (auto* obj: objects_) {
        for (auto *cell: obj->footprint()) {
            *cell = obj->cellType();
        }
    }
}

std::pair<int,int> Map2D::worldToCell(float world_x, float world_y) const
{
    int pixel_x = static_cast<int>(world_x / resolution_ + resolution_);
    int pixel_y = static_cast<int>(world_y / resolution_ + resolution_);
    return { std::clamp(pixel_x, 0, width_  - 1),
         std::clamp(pixel_y, 0, height_ - 1) };
}

void Map2D::addObject(MapObject& object)
{
    objects_.push_back(&object);
    object.setMap(this);  // set the map for the object
}

void Map2D::startSimulation() {
    if (simulation_thread_.joinable()) {
        logger().warn("Simulation already active. Ending previous simulation.");
        endSimulation();
    }

    capture_frames_.clear();
    simulation_active_ = true;

    // starts a thread that captures frames every update_period_ milliseconds
    simulation_thread_ = std::thread([this]() {
        // start updating all objects
        unsigned int num_frames = 0;
        base_data_ = map_data_;
        const auto period = std::chrono::duration<float>(constants::step_size);
        while (simulation_active_ && num_frames < max_frames_stored_)
            {
                for (auto* obj : objects_) obj->update();
                using namespace std::chrono_literals;
                this->removeFootprints();
                this->addMotionFrame();  // capture the current frame
                std::this_thread::sleep_for(period);
                ++num_frames;
                // swaps buffers
                std::ranges::copy(base_data_, map_data_.begin());
            }
        if (num_frames > max_frames_stored_) {
            logger().warn("Maximum number of frames stored reached: {}. Stopping simulation.", max_frames_stored_);
        }
        simulation_active_ = false;
    });
}

void Map2D::addMotionFrame()
{
    if (!simulation_active_) return;
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

void Map2D::endSimulation() {
    simulation_active_ = false;
    if (simulation_thread_.joinable()) simulation_thread_.join();
    if (capture_frames_.empty()) return;
}


void Map2D::startSimulationFor(unsigned int seconds)
{
    if (simulation_thread_.joinable()) {
        logger().warn("Simulation already active. Ending previous simulation.");
        endSimulation();
    }

    capture_frames_.clear();
    simulation_active_ = true;
    base_data_ = map_data_;

    simulation_thread_ = std::thread([this, seconds]() {
        const auto period = std::chrono::duration<float>(constants::step_size);
        auto start_time = std::chrono::steady_clock::now();

        while (simulation_active_) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time
            ).count();
            if (elapsed >= seconds) break;
            for (auto* obj : objects_) obj->update();
            this->removeFootprints();
            this->addMotionFrame();
            std::this_thread::sleep_for(period);
            // swaps buffers
            std::ranges::copy(base_data_, map_data_.begin());
        }
        simulation_active_ = false;   // auto-stop
    });
}

void Map2D::addMotionFrame(const cv::Mat& annotated)
{
    if (!simulation_active_) {
        throw std::runtime_error("Motion capture is not active. Call startMotionCapture() first.");
    };

    capture_frames_.push_back(annotated.clone());
}

void Map2D::flushFrames(const std::string& filename)
{
    if (capture_frames_.empty()) {
        logger().warn("No frames captured. Skipping video creation.");
        return;
    }

    cv::VideoWriter w(filename,
                      cv::VideoWriter::fourcc('m','p','4','v'),
                      constants::FPS,
                      capture_frames_[0].size());

    for (const auto& f : capture_frames_) w.write(f);
    w.release();

    capture_frames_.clear();
    logger().info("Video saved to {}", filename);
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










