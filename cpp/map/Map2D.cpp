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
#include "../objects/Vehicle.h"
#include "../objects/Road.h"
#include "../objects/Grass.h"
#include "../objects/Obstacle.h"
#include "../objects/Free.h"
#include "map/Map2D.h"

using json = nlohmann::json;
namespace fs = std::filesystem;
fs::path cell_color_data_json = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/cell_colors.json";
fs::path size_cfg_path = "/home/critter/workspace/autonomous_vehicle_simulator/res/objects_size.json";

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
    Map2D local_map =  Map2D(w_window, h_window);

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

Map2D::Map2D(int width, int height) {
    if (width > MAX_MAP_WIDTH || height > MAX_MAP_HEIGHT) {
        throw std::out_of_range(std::format(
        "Map size exceeds maximum dimensions. Max width: {}, max height: {}", MAX_MAP_WIDTH, MAX_MAP_HEIGHT));
    }
    width_ = width;
    height_ = height;
    map_data_.resize(width * height);
    objects_.clear();
    frame_cache_valid_ = false;

    // Create a giant Free object that fills the entire map
    float map_width_meters = width * resolution_;
    float map_height_meters = height * resolution_;
    background_free_object_ = std::make_unique<Free>(map_width_meters, map_height_meters,
                                       map_width_meters/2, map_height_meters/2, 0.0f);
    // Create a copy for the objects vector
    addObject(std::move(background_free_object_));
}

Map2D Map2D::loadMap(const std::string &map_file,
            const std::string& metadata_json_file) {
    int w, h;
    std::vector<std::array<uint8_t, 3>> bmp = utils::loadBmp(map_file, w, h);
    logger().info("Loaded map from file: {}. Size: {}x{}", map_file, w, h);
    Map2D m(w, h);
    m.fillMapWith(Cell::Unknown);

    fs::path imagePath(map_file);
    std::ifstream metaFile(metadata_json_file);
    if (!metaFile) {
        logger().error("Metadata file not found at {}", metadata_json_file);
        throw std::runtime_error("Missing metadata.json for map loading");
    }
    nlohmann::json meta;
    metaFile >> meta;
    std::ifstream cfgFile(size_cfg_path);
    nlohmann::json sizeCfg;
    if (!cfgFile) {
        logger().error("Object sizes config not found at {}", size_cfg_path.string());
        throw std::runtime_error("Missing objects_size.json for map loading");
    }
    cfgFile >> sizeCfg;
    // Support both formats: metadata could be an array or under "objects"
    const auto &objects = meta.contains("objects") ? meta["objects"] : meta;
    for (const auto& obj : objects) {

        std::string type = obj.at("type");
        if (!sizeCfg.contains(type)) {
            throw std::runtime_error("sizeCfg missing geometry for type: " + type);
        }

        int px = obj.at("x");
        int py = obj.at("y");
        float rot_deg = obj.at("rotation");
        float L = sizeCfg.at(type).at("length");
        float W = sizeCfg.at(type).at("width");
        float rot_rad = rot_deg * M_PI / 180.0f;
        float world_x = px * m.resolution_;
        float world_y = py * m.resolution_;
        std::unique_ptr<MapObject> mapobj = nullptr;

        if (type == "Obstacle") {
            mapobj = std::make_unique<Obstacle>(L, W, world_x, world_y, rot_rad);
        } else if (type == "Grass") {
            mapobj = std::make_unique<Grass>(L, W, world_x, world_y, rot_rad);
        } else if (type == "Road") {
            mapobj = std::make_unique<Road>(L, W, world_x, world_y, rot_rad);
        } else if (type == "Vehicle") {
            float motorForce = 0.0f;
            if (sizeCfg["Vehicle"].contains("motor_force")) {
                motorForce = sizeCfg["Vehicle"]["motor_force"];
            }
            std::array<uint8_t,3> color_bgr = cellToColor(Cell::Vehicle);
            mapobj = std::make_unique<Vehicle>(L, W, color_bgr, rot_rad, 0.0f, world_x, world_y, motorForce);
        }
        if (mapobj) {
            m.addObject(std::move(mapobj));
        }
    }
    logger().info("Loaded {} objects from metadata {}", objects.size(), metadata_json_file);
    return m;
}
Map2D& globalMap() {
    static Map2D map = Map2D::loadMap(DEFAULT_MAP_FILE, DEFAULT_METADATA_FILE);
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

void Map2D::addObject(std::unique_ptr<MapObject> object)
{
    if (object->bodyType() == b2_staticBody) {
        stampObject(object);
    }
    object->setMap(this);  // set the map for the object
    objects_.push_back(std::move(object));
}

void Map2D::startAllObjects() const {
    logger().info("Starting all {} objects in the map", objects_.size());
    for (const auto& obj : objects_) {
        if (obj) {
            obj->start();
        }
    }
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
                for (auto& obj : objects_) {
                    if (!objectIsRemoved_(obj.get())) {
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
    for (const auto& obj : objects_) {
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

void Map2D::stampObject(const std::unique_ptr<MapObject>& object)
{
    // world -> pixel polygon
    const auto corners = object->worldBoxCorners();
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

    const Cell c = object->cellType();
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

bool Map2D::objectIsInMap_( MapObject* obj) const
{
    const auto corners = obj->worldBoxCorners();
    std::array<cv::Point,4> poly{};
    for (size_t i = 0; i < 4; ++i) {
        poly[i].x = std::lround(corners[i].x / resolution_);
        poly[i].y = std::lround(corners[i].y / resolution_);
    }

    const cv::Rect frame(0, 0, width_, height_);
    const cv::Rect bb   = cv::boundingRect(std::vector<cv::Point>(poly.begin(), poly.end()));
    return ( (bb & frame).area() > 0 );
}

void Map2D::removeObject_(std::unique_ptr<MapObject> obj)
{
    if (!obj) return;

    if (obj.get() == background_free_object_.get()) {
        logger().warn("Attempted to remove protected background Free object. Ignoring deletion request.");
        return;
    }

    offmap_time_.erase(obj.get());

    // Remove from objects_ vector onl if the object  can be found in vector objects_
    auto it = std::ranges::find_if(objects_,
                                   [&obj](const std::unique_ptr<MapObject>& ptr) {
                                       return ptr.get() == obj.get();
                                   });

    if (it != objects_.end()) {
        objects_.erase(it);
    }

    // obj will be automatically deleted when unique_ptr goes out of scope
    logger().debug("Removed object {} from map.", static_cast<const void*>(obj.get()));
}

bool Map2D::objectIsRemoved_(MapObject* obj) {
    if ( objectIsInMap_(obj)) {
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
        // Find the object in the vector and remove it
        auto obj_it = std::find_if(objects_.begin(), objects_.end(),
            [obj](const std::unique_ptr<MapObject>& ptr) {
                return ptr.get() == obj;
            });

        if (obj_it != objects_.end()) {
            auto obj_to_remove = std::move(*obj_it);
            removeObject_(std::move(obj_to_remove));
        }

        logger().info("Object {} removed after being off-map for {:.1f} seconds.", static_cast<const void*>(obj), dt);
        return true;
    }
    return false;
}

void Map2D::destroyAllDynamicObjects() {
    logger().info("Destroying all {} objects in the map", objects_.size());
    auto it = objects_.begin();
    while (it != objects_.end()) {
        if (*it && (*it)->bodyType() == b2_dynamicBody) {
            auto obj_to_remove = std::move(*it);
            it = objects_.erase(it); // erase returns iterator to next element
            removeObject_(std::move(obj_to_remove));
        } else {
            ++it;
        }
    }
    invalidateFrameCache();
}
