#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include "../utils/logging.h"
#include "../data/constants.h"
#include "../include/nlohmann/json.hpp"
#include "../objects/MapObject.h"
#include "../objects/vehicle/Vehicle.h"
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

std::vector<std::array<uint8_t, 3>> Map2D::toBmp(std::vector<Cell> cell_data) {
    std::vector<std::array<uint8_t, 3>> bmp_matrix;
    for (const auto& cell : cell_data) {
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
    cf_chunk_ = std::max<size_t>(1, max_frames_stored_ / 10);
    // for conenience and the ring buffer, use this simple rule!
    if (max_frames_stored_ % 10 != 0) {
        throw std::runtime_error("max_frames_stored_ must be a multiple of 10");
    }

    // Create a giant Free object that fills the entire map
    float map_width_meters = width * resolution_;
    float map_height_meters = height * resolution_;
    auto free_object = std::make_unique<Free>(map_width_meters, map_height_meters,
                                       map_width_meters/2, map_height_meters/2, 0.0f);
    background_free_object_ = free_object.get(); // warning "the address of the local variable 'free_object' may escape the function" can be ignored because
                                                // the object is added to the map, hence the map keeps it alive
    // move the background_free_
    addObject(std::move(free_object));
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
            obj->startUpdating();
        }
    }
}

#ifdef FRAME_PROFILER
#include "../utils/Profiler.h"

void Map2D::startUpdatingObjectsPhysics_() {
    physics_update_thread = std::thread([this]() {
        FrameProfiler physics_prof("PhysicsProfiler");
        const auto period = std::chrono::duration<float>(constants::step_size);
        while (simulation_active_) {
            physics_prof.begin_frame();
            auto start = std::chrono::steady_clock::now();
            {
                ScopedSection sec(physics_prof, "Update World");
                updateWorld(start);
            }
            {
                ScopedSection sec(physics_prof, "UpdateObjects");
                for (auto& obj : objects_) {
                    objectOffMapTooLong_(obj.get());
                    if (obj != nullptr) {
                        if (obj->isStarted()) {
                            obj->update();
                        } else {
                            obj->freeze();
                        }
                    }
                    else {
                        removeObject_(std::move(obj));
                    }
                }
            }
            const auto end = std::chrono::steady_clock::now();
            const auto elapsed = end - start;
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

            std::chrono::steady_clock::duration sleep_time{};
            if (elapsed > period) {
                WARN_ONCE(
                     std::string("Update physics step step took ")
                     + std::to_string(elapsed_ms)
                     + " ms, which is longer than the target step size of "
                     + std::to_string(constants::step_size * 1000.0)
                     + " ms."
                     );
                sleep_time = std::chrono::steady_clock::duration::zero();
            } else {
                sleep_time = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period - elapsed);
            }
            physics_prof.end_frame();
            std::this_thread::sleep_for(sleep_time);
        }
    });
}

void Map2D::startFrameDecoderThread_() {
    capture_frames_.clear();
    simulation_active_ = true;
    decoder_thread = std::thread([this]() {
        FrameProfiler decoder_prof("FrameDecoder");
        unsigned int num_frames = 0;
        const auto period = std::chrono::duration<float>(constants::step_size);
        while (simulation_active_) {
            decoder_prof.begin_frame();
            if (num_frames > max_frames_stored_) {
                logger().warn("Maximum number of frames stored reached: {}. Stopping simulation.", max_frames_stored_);
                return;
            }
            auto start = std::chrono::steady_clock::now();
             {
                 ScopedSection sec(decoder_prof, "Add Motion Frame");
                 this->addMotionFrame();
             }
            const auto end = std::chrono::steady_clock::now();
            const auto elapsed = end - start;
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

            std::chrono::steady_clock::duration sleep_time{};
            if (elapsed > period) {
                WARN_ONCE(
                     std::string("Frame decoding step took ")
                     + std::to_string(elapsed_ms)
                     + " ms, which is longer than the target step size of "
                     + std::to_string(constants::step_size * 1000.0)
                     + " ms."
                     );
                sleep_time = std::chrono::steady_clock::duration::zero();
            } else {
                sleep_time = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period - elapsed);
            }
            decoder_prof.end_frame();
            std::this_thread::sleep_for(sleep_time);
        }
    });
}

#else

void Map2D::startUpdatingObjectsPhysics_() {
    physics_update_thread = std::thread([this]() {
        const auto period = std::chrono::duration<float>(constants::step_size);
        while (simulation_active_) {
            auto start = std::chrono::steady_clock::now();

            for (auto& obj : objects_) {
                if (!objectOffMapTooLong_(obj.get())) {
                    if (obj->isStarted()) {
                        obj->update();
                    } else {
                        obj->freeze();
                    }
                }
                else {
                    removeObject_(std::move(obj));
                }
            }
            updateWorld(start);
            const auto end = std::chrono::steady_clock::now();
            const auto elapsed = end - start;
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

            std::chrono::steady_clock::duration sleep_time{};
            if (elapsed > period) {
                // Keep this cheap: avoid building big strings every frame.
                // Use your logger's formatting instead of concatenation if you can.
                WARN_ONCE(
                     std::string("Update physics step step took ")
                     + std::to_string(elapsed_ms)
                     + " ms, which is longer than the target step size of "
                     + std::to_string(constants::step_size * 1000.0)
                     + " ms."
                     );
                sleep_time = std::chrono::steady_clock::duration::zero();
            } else {
                sleep_time = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period - elapsed);
            }
            std::this_thread::sleep_for(sleep_time);

        }
    });
}

void Map2D::startFrameDecoderThread_() {
    capture_frames_.clear();
    simulation_active_ = true;
    decoder_thread = std::thread([this]() {
        unsigned int num_frames = 0;
        const auto period = std::chrono::duration<float>(constants::step_size);
        while (simulation_active_) {
            if (num_frames > max_frames_stored_) {
                logger().warn("Maximum number of frames stored reached: {}. Stopping simulation.", max_frames_stored_);
                return;
            }
            auto start = std::chrono::steady_clock::now();
            this->addMotionFrame();
            const auto end = std::chrono::steady_clock::now();
            const auto elapsed = end - start;
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

            std::chrono::steady_clock::duration sleep_time{};
            if (elapsed > period) {
                WARN_ONCE(
                     std::string("Update physics step step took ")
                     + std::to_string(elapsed_ms)
                     + " ms, which is longer than the target step size of "
                     + std::to_string(constants::step_size * 1000.0)
                     + " ms."
                     );
                sleep_time = std::chrono::steady_clock::duration::zero();
            } else {
                sleep_time = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period - elapsed);
            }
            std::this_thread::sleep_for(sleep_time);
        }
    });
}
#endif

void Map2D::stopUpdatingObjectsPhysics_() {
    if (physics_update_thread.joinable()) {
        physics_update_thread.join();
    }
}

void Map2D::startSimulation() {
    if (simulation_active_) {
        logger().warn("Simulation already active. Ending previous simulation.");
        endSimulation();
    }
    startFrameDecoderThread_();
    startUpdatingObjectsPhysics_();
}

void Map2D::stopDecoder_() {
    if (decoder_thread.joinable()) {
        decoder_thread.join();
    }
}

void Map2D::initializeCachedFrame() const {
    if (frame_cache_valid_) return;

    cached_original_frame_ = Frame(height_, width_, CV_8UC3);
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

void Map2D::addMotionFrame() {
    initializeCachedFrame();
    // expand the buffer if needed
    ensureCaptureCapacityForNextPush_();

    // Copy the cached original frame
    cv::Mat frame = cached_original_frame_.clone();

    // Stamp dynamic objects onto the frame
    stampObjectsOntoFrame_(frame);

    if (cf_size_ >= cf_capacity_) {
        throw std::runtime_error("Capture buffer overflow");
    }
    const size_t pos = cf_head_ + cf_size_;
    capture_frames_[pos] = std::move(frame);
    ++cf_size_;
}

void Map2D::invalidateFrameCache() const {
    frame_cache_valid_ = false;
}

// New method: stamps all dynamic objects onto the given frame
void Map2D::stampObjectsOntoFrame_(cv::Mat& frame) const
{
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
}

void Map2D::endSimulation() {
    simulation_active_ = false;
    stopDecoder_();
    stopUpdatingObjectsPhysics_();
}

void Map2D::addMotionFrame(const Frame &annotated)
{
    initializeCachedFrame();
    // expand the buffer if needed
    ensureCaptureCapacityForNextPush_();
    const size_t pos = cf_head_ + cf_size_;
    capture_frames_[pos] = std::move(annotated);
    ++cf_size_;
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
    return (bb & frame).area() > 0;
}

void Map2D::removeObject_(std::unique_ptr<MapObject> obj)
{
    if (!obj) return;

    if (obj.get() == background_free_object_) {
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
        // obj will be automatically deleted when unique_ptr goes out of scope
        logger().debug("Removed object {} from map.", static_cast<const void*>(obj.get()));
    }
}

bool Map2D::objectOffMapTooLong_(MapObject* obj) {
    if (obj == nullptr) {
        return false;
    }
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
        return true; // signal to remove
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

void Map2D::ensureCaptureCapacityForNextPush_()
{
    // Initial setup: allocate first chunk of slots.
    if (cf_capacity_ == 0) {
        cf_capacity_ = std::min(max_frames_stored_, cf_chunk_);
        capture_frames_.resize(cf_capacity_);  // default-constructed cv::Mat slots
        cf_head_ = 0;
        cf_size_ = 0;
        return;
    }

    // Still have a free slot? nothing to do.
    if (cf_size_ < cf_capacity_) return;

    // Full: try to grow if we can.
    if (cf_capacity_ < max_frames_stored_) {
        // Geometric growth (at least +cf_chunk_), clamped to max_frames_stored_.
        size_t proposed = std::max(cf_capacity_ * 2, cf_capacity_ + cf_chunk_);
        size_t new_cap  = std::min(proposed, max_frames_stored_);

        // Fast path: already linear [0..cf_size_)
        if (cf_head_ == 0) {
            // Reserve first to reduce chances of reallocation; then resize to create new slots.
            capture_frames_.reserve(new_cap);
            capture_frames_.resize(new_cap);
            cf_capacity_ = new_cap;
            return;
        }

        // General path: ring is split into two linear segments.
        // Build a new storage, move in two ranges, then pad to new_cap.
        std::vector<cv::Mat> newStore;
        newStore.reserve(new_cap);

        // First segment: [cf_head_ .. cf_head_ + first)
        const size_t first = std::min(cf_size_, cf_capacity_ - cf_head_);
        newStore.insert(newStore.end(),
                        std::make_move_iterator(capture_frames_.begin() + cf_head_),
                        std::make_move_iterator(capture_frames_.begin() + cf_head_ + first));

        // Second segment (if any): [0 .. second)
        const size_t second = cf_size_ - first;
        if (second) {
            newStore.insert(newStore.end(),
                            std::make_move_iterator(capture_frames_.begin()),
                            std::make_move_iterator(capture_frames_.begin() + second));
        }

        // Pad with empty slots to reach new_cap (keeps index-by-slot semantics).
        newStore.resize(new_cap);

        capture_frames_.swap(newStore);
        cf_capacity_ = new_cap;
        cf_head_     = 0;          // now linearized
        // cf_size_ unchanged
    }
}


void Map2D::buildGraph_() {
    resetGraphStorage_();
    const auto roads = collectRoads_();
    if (roads.empty()) return;

    std::unordered_map<Road*, std::vector<int>> roadNodes;
    addIntersectionNodes_(roads, roadNodes);
    addEndpointNodes_(roads, roadNodes);
    addEdgesAlongRoads_(roads, roadNodes);
}

void Map2D::resetGraphStorage_() {
    global_graph_ = Graph<int>();
    node_coords_.clear();
}

std::vector<Road*> Map2D::collectRoads_() const {
    std::vector<Road*> out;
    out.reserve(objects_.size());
    for (auto& o : objects_) {
        if (!dynamic_cast<Road*>(o.get())) continue;
        out.push_back(static_cast<Road*>(o.get()));
    }
    return out;
}

int Map2D::addNode_(const b2Vec2& p) {
    //tODO: return the existing node if p is very close to an existing node
    int id = static_cast<int>(graph_nodes_.size());
    std::string label = "lbl_id" + std::to_string(id);
    auto new_node = std::make_unique<Node<int>>(label, id);
    auto* node_ptr = new_node.get();
    graph_nodes_.push_back(std::move(new_node));
    node_coords_.push_back(p);
    global_graph_.addNode(node_ptr);
    return id;
}

std::array<b2Vec2,4> Map2D::roadCorners_(const Road* r) const {
    const b2Vec2 C = r->position();
    const float a = b2Rot_GetAngle(r->rotation());
    const b2Vec2 d{std::cos(a), std::sin(a)};
    const b2Vec2 n{-d.y, d.x};
    const float L = r->length(), W = r->width();
    return { C + 0.5f*L*d + 0.5f*W*n,
             C + 0.5f*L*d - 0.5f*W*n,
             C - 0.5f*L*d + 0.5f*W*n,
             C - 0.5f*L*d - 0.5f*W*n };
}

bool Map2D::obbOverlap_(const std::array<b2Vec2,4>& A, const std::array<b2Vec2,4>& B) {
    const auto axes = [&](){
        std::array<b2Vec2,4> ax;
        ax[0] = {A[0].x - A[1].x, A[0].y - A[1].y};
        ax[1] = {A[0].x - A[2].x, A[0].y - A[2].y};
        ax[2] = {B[0].x - B[1].x, B[0].y - B[1].y};
        ax[3] = {B[0].x - B[2].x, B[0].y - B[2].y};
        return ax;
    }();
    for (auto a : axes) {
        const float len = std::hypot(a.x, a.y);
        if (len < 1e-6f) continue;
        a.x/=len; a.y/=len;
        auto [m1, M1] = utils::projectMinMaxPolygon(A,a);
        auto [m2, M2] = utils::projectMinMaxPolygon(B,a);
        if (M1 < m2 - 1e-4f || M2 < m1 - 1e-4f) return false;
    }
    return true;
}

bool Map2D::centerlineCrossPoint_(const Road* r1, const Road* r2, b2Vec2& p) {
    const b2Vec2 C1 = r1->position(), C2 = r2->position();
    const float a1 = b2Rot_GetAngle(r1->rotation());
    const float a2 = b2Rot_GetAngle(r2->rotation());
    const b2Vec2 d1{std::cos(a1), std::sin(a1)};
    const b2Vec2 d2{std::cos(a2), std::sin(a2)};
    const float det = d1.x*d2.y - d1.y*d2.x;
    if (std::fabs(det) < 1e-6f) return false;
    const b2Vec2 dv = C2 - C1;
    const float t = (dv.x*d2.y - dv.y*d2.x) / det;
    const float u = (dv.x*d1.y - dv.y*d1.x) / det;
    const float L1 = 0.5f * r1->length(), L2 = 0.5f * r2->length();
    if (std::fabs(t) > L1 + 1e-3f || std::fabs(u) > L2 + 1e-3f) return false;
    p = C1 + t*d1;
    return true;
}

void Map2D::addIntersectionNodes_(
    const std::vector<Road*>& roads,
    std::unordered_map<Road*, std::vector<int>>& roadNodes)
{
    for (size_t i=0;i<roads.size();++i){
        auto r1 = roads[i];
        const auto c1 = roadCorners_(r1);
        for (size_t j=i+1;j<roads.size();++j){
            auto r2 = roads[j];
            const auto c2 = roadCorners_(r2);
            if (!obbOverlap_(c1,c2)) continue;
            b2Vec2 P;
            if (!centerlineCrossPoint_(r1,r2,P)) {
                // fallback: use nearer endpoint as junction
                const b2Vec2 C1=r1->position(), C2=r2->position();
                const b2Vec2 d1 = c1[0]; (void)d1; // hint: corners already imply overlap
                P = 0.5f*(C1+C2);
            }
            const int nid = addNode_(P);
            roadNodes[r1].push_back(nid);
            roadNodes[r2].push_back(nid);
        }
    }
}

void Map2D::addEndpointNodes_(
    const std::vector<Road *> &roads,
    std::unordered_map<Road *, std::vector<int> > &roadNodes) {
    for (auto *r: roads) {
        auto it = roadNodes.find(r);
        const bool hasAny = (it != roadNodes.end() && !it->second.empty());
        const b2Vec2 C = r->position();
        const float a = b2Rot_GetAngle(r->rotation());
        const b2Vec2 d{std::cos(a), std::sin(a)};
        const b2Vec2 e1 = C + 0.5f * r->length() * d;
        const b2Vec2 e2 = C - 0.5f * r->length() * d;

        if (!hasAny) {
            roadNodes[r] = {addNode_(e1), addNode_(e2)};
            continue;
        }
        if (roadNodes[r].size() == 1) {
            const b2Vec2 ex = node_coords_[roadNodes[r][0]];
            const b2Vec2 cand = (b2Distance(ex, e1) > b2Distance(ex, e2)) ? e1 : e2;
            roadNodes[r].push_back(addNode_(cand));
        }
    }
}

void Map2D::addEdgesAlongRoads_(
    const std::vector<Road *> &roads,
    const std::unordered_map<Road *, std::vector<int> > &roadNodes) {
    for (auto *r: roads) {
        auto it = roadNodes.find(r);
        if (it == roadNodes.end() || it->second.size() < 2) continue;

        auto ids = it->second;
        const b2Vec2 C = r->position();
        const float a = b2Rot_GetAngle(r->rotation());
        const b2Vec2 d{std::cos(a), std::sin(a)};
        std::sort(ids.begin(), ids.end(), [&](int A, int B) {
            const float ta = b2Dot(node_coords_[A] - C, d);
            const float tb = b2Dot(node_coords_[B] - C, d);
            return ta < tb;
        });
        for (size_t k = 1; k < ids.size(); ++k) {
            const int u = ids[k - 1], v = ids[k];
            Node<int>* node1 = graph_nodes_[u].get();
            Node<int>* node2 = graph_nodes_[v].get();
            std::pair node_pair_12 = {node1, node2};
            std::pair node_pair_21 = {node2, node1};
            CXXGraph::id_t edge_id = graph_edges_.size();
            auto edge_12 = std::make_unique<Edge<int>>(edge_id, node_pair_12);
            auto edge_21 = std::make_unique<Edge<int>>(edge_id, node_pair_21);
            global_graph_.addEdge(edge_12.get());
            global_graph_.addEdge(edge_21.get());
            graph_edges_.push_back(std::move(edge_12));
            graph_edges_.push_back(std::move(edge_21));
        }
    }
}
