#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include "../data/constants.h"
#include "../include/nlohmann/json.hpp"
#include "../objects/MapObject.h"
#include "map/Map2D.h"

using json = nlohmann::json;
namespace fs = std::filesystem;
fs::path cell_color_data_json = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/cell_colors.json";

struct CellColorLoader {
    CellColorLoader() {
        loadCellColors(cell_color_data_json);
    }
};


// this class was created only so that the cell colors are loaded as soon as the program starts
static CellColorLoader init_cell_colors;

Cell Map2D::atPx(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range(std::format(
        "Coordinates out of bounds. Max x range: {}, max y range: {}", width_, height_));
    };
    return map_data_[idx(x, y)];
}

void Map2D::setPx(int x, int y, Cell cell) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        throw std::out_of_range(std::format(
        "Coordinates out of bounds. Max x range: {}, max y range: {}", width_, height_));
    };
    map_data_[idx(x, y)] = cell;
    // Invalidate cache when static map data changes
    invalidateFrameCache();
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

    return local_map;
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
    objects_.clear();
    frame_cache_valid_ = false;
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
    if (object.bodyType() == b2_staticBody) {
        stampObject(object);
        return;
    }
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
        const auto period = std::chrono::duration<float>(constants::step_size);
        while (simulation_active_ && num_frames < max_frames_stored_)
            {
                for (MapObject* obj : objects_) {
                    if (!objectIsRemoved_(obj)) {
                        if (obj->isStarted()) {
                            obj->update();
                        }
                        else {
                            obj->freeze();
                        }
                    }
                }
                b2World_Step(WORLD, constants::step_size, 4);
                // bounce response (perfectly rigid bodies)
                using namespace std::chrono_literals;
                this->addMotionFrame();  // capture the current frame
                std::this_thread::sleep_for(period);
                ++num_frames;
                // swaps buffers
            }
        if (num_frames > max_frames_stored_) {
            logger().warn("Maximum number of frames stored reached: {}. Stopping simulation.", max_frames_stored_);
        }
        simulation_active_ = false;
    });
}

void Map2D::initializeCachedFrame() const {
    if (frame_cache_valid_) return;

    cached_original_frame_ = cv::Mat(height_, width_, CV_8UC3);
    const auto bmp = toBmp();

    // TODO: parallelize with OpenMP
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const auto& c = bmp[y * width_ + x]; // RGB → BGR
            cached_original_frame_.at<cv::Vec3b>(y, x) = { c[2], c[1], c[0] };
        }
    }
    frame_cache_valid_ = true;
}

void Map2D::invalidateFrameCache() const {
    frame_cache_valid_ = false;
}

void Map2D::addMotionFrame()
{
    if (!simulation_active_) return;

    // Ensure cached frame is valid
    initializeCachedFrame();

    // Copy the cached original frame
    cv::Mat frame = cached_original_frame_.clone();

    // 2) overlay each dynamic object's current polygon from Box2D
    for (const MapObject* obj : objects_) {
        if (obj->bodyType() == b2_staticBody) continue; // skip static objects
        const auto corners = obj->worldBoxCorners();

        std::array<cv::Point, 4> poly{};
        for (size_t i = 0; i < 4; ++i) {
            // world meters -> pixel coords
            const int ix = std::lround(corners[i].x / resolution_);
            const int iy = std::lround(corners[i].y / resolution_);

            poly[i].x = ix;
            poly[i].y = iy;
        }

        const auto bgr = obj->colorBGR();
        cv::fillConvexPoly(
            frame,
            poly.data(),
            static_cast<int>(poly.size()),
            cv::Scalar(bgr[0], bgr[1], bgr[2]),
            cv::LINE_AA
        );
    }
    capture_frames_.push_back(std::move(frame));
}

void Map2D::endSimulation() {
    simulation_active_ = false;
    if (simulation_thread_.joinable()) simulation_thread_.join();
    if (capture_frames_.empty()) return;
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
    // Use cached frame if available
    initializeCachedFrame();
    return cached_original_frame_.clone();
}

void Map2D::stampObject(const MapObject& object)
{
    // world -> pixel polygon
    const auto corners = object.worldBoxCorners();
    std::array<cv::Point,4> poly{};
    for (size_t i = 0; i < 4; ++i) {
        const int ix = std::lround(corners[i].x / resolution_);
        const int iy = std::lround(corners[i].y / resolution_);
        poly[i].x = ix;
        poly[i].y = iy;
    }

    // draw into mask and write back to map_data_
    cv::Mat mask(height_, width_, CV_8UC1, cv::Scalar(0));
    cv::fillConvexPoly(mask, poly.data(), (int)poly.size(), cv::Scalar(255), cv::LINE_AA);

    const Cell c = object.cellType();
    for (int y = 0; y < height_; ++y) {
        const uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < width_; ++x) {
            if (row[x]) {
                map_data_[idx(x,y)] = c; // write to base map
            }
        }
    }
    invalidateFrameCache(); // refresh cached original frame
}

bool Map2D::objectIsInMap_(const MapObject& obj) const
{
    const auto corners = obj.worldBoxCorners();
    std::array<cv::Point,4> poly{};
    for (size_t i = 0; i < 4; ++i) {
        poly[i].x = std::lround(corners[i].x / resolution_);
        poly[i].y = std::lround(corners[i].y / resolution_);
    }

    const cv::Rect frame(0, 0, width_, height_);
    const cv::Rect bb   = cv::boundingRect(std::vector<cv::Point>(poly.begin(), poly.end()));
    return ( (bb & frame).area() > 0 );
}


void Map2D::removeObject_(MapObject* obj)
{
    if (!obj) return;
    std::erase(objects_, obj);
    offmap_time_.erase(obj);

    logger().info("Removed object {} from map and world after off-map TTL of {}.", static_cast<const void*>(obj), constants::max_offmap_time);
}

bool Map2D::objectIsRemoved_(MapObject* obj) {
    if (objectIsInMap_(*obj)) {
        offmap_time_.erase(obj);
        return false;
    }
    auto now = std::chrono::steady_clock::now();
    auto it = offmap_time_.find(obj);
    if (it == offmap_time_.end()) {
        offmap_time_.emplace(obj, now);
        logger().debug("Object {} went off-map; starting TTL.",
                       static_cast<const void*>(obj));
        return false;
    }
    const float dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - it->second).count();
    if (dt >= constants::max_offmap_time) {
        removeObject_(obj);
        logger().info("Object {} removed after being off-map for {:.1f} seconds.", static_cast<const void*>(obj), dt);
        return true;
    }
    return false;
}
