#ifndef MAP2D_H
#define MAP2D_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include <unordered_map>
#include <box2d/box2d.h>
#include <include/CXXGraph/CXXGraph.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "../data/DataClasses.h"
#include "../utils/GraphUtils.hpp"
#include "../utils/LoggingUtils.hpp"
#include "../utils/ImageUtils.hpp"
#include "../objects/MapObject.h"


constexpr int MAX_MAP_WIDTH = 2000;
constexpr int MAX_FRAMES_STORED = 100000; // max number of frames to store in memory for video capture
constexpr int MAX_MAP_HEIGHT = 2000;
constexpr double DEFAULT_MAP_RESOLUTION = 0.1; // meters/pixel
// const std::string DEFAULT_MAP_FILE = "../../res/maps/default_map.bmp"; // TODO: find a way to make relative path to work again
const std::string DEFAULT_MAP_FILE = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/map.bmp";
const std::string DEFAULT_METADATA_FILE = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/map.json";


// Global concurrency primitives
inline std::thread physics_update_thread;
inline std::thread decoder_thread;

// Global decoder queue
class Vehicle;
class Road;

using namespace CXXGraph;
using Frame = cv::Mat;

class Map2D {
public:
    /**
     * @brief Construct a new Map2D object. Upon creation, a "Free" object
     * occupies the entire map area.
     *
     * @param width Width of the map in pixels
     * @param height Height of the map in pixels
     */
    Map2D(
    int width,
    int height
    );

    // Enable copy semantics (reset to default)
    Map2D(const Map2D&) = default;
    Map2D& operator=(const Map2D&) = default;

    // Reset move semantics to default
    Map2D(Map2D&& other) = default;
    Map2D& operator=(Map2D&& other) = default;

    /**
     * @brief Destructor. Remove all objects from the map.
     */
    ~Map2D() {
        endSimulation();
        offmap_time_.clear();
        objects_.clear();
        background_free_object_ = nullptr;
    };

    /**
     * @brief  Returns the cell class at the given coordinates. /TODO: add bounds checking
     *
     * @param x X coordinate in pixels
     * @param y Y coordinate in pixels
     *
     * @return Cell The cell class at the given coordinates
     */
    [[nodiscard]] Cell atPx(int x, int y) const;

    /**
     * @brief Sets the cell class at the given coordinates.
     * 
     * @param x X coordinate in pixels
     * @param y Y coordinate in pixels
     * @param value The cell class to set at the given coordinates
     */
    void setPx(int x, int y, Cell value);

    /**
     * @brief  Returns a local squared Map2D object that surrounds the vehicle. The square is rotated to
     * the vehicle's orientation.
     *
     * @param cx X coordinate of the vehicle's center in pixels
     * @param cy Y coordinate of the vehicle's center in pixels
     * @param radius_px Radius in pixels
     *
     * @return Map2D A local map centered around the vehicle
     */
    [[nodiscard]] Map2D window(int cx, int cy, int radius_px) const;

    /**
     * @brief Load a map from a .bmp file and its associated metadata from a .json file.
     *
     * The metadata file should have this form: \n
     * ```json
     * {
     *  "Vehicle": {"length": 4.5, "width": 2.0, "motor_force": 8000},
     *  "Obstacle": {"length": 1.0, "width": 1},
     *  "Grass": {"length": 2.0, "width": 2.0}
     *  }
     *  ```
     *
     *  Note that the keys in the metadata file have to match the cell type names
     *
     * @param map_file Path to the .bmp file
     * @param metadata_json_file Path to the .json file containing the object metadata
     *
     * @return Map2D The loaded map with objects placed according to the metadata
     *
     * @throws std::runtime_error if the metadata or size config file could not be read
     */
    static Map2D loadMap(const std::string& map_file, const std::string& metadata_json_file);

    //getters
    [[nodiscard]] int width() const noexcept {return width_;}
    [[nodiscard]] int height() const noexcept {return height_;}
    [[nodiscard]] double resolution() const noexcept {return resolution_;}
    [[nodiscard]] const std::vector<std::unique_ptr<MapObject>>& objects() const noexcept { return objects_; }
    [[nodiscard]] unsigned int maxFrameStored() const noexcept { return max_frames_stored_; }
    [[nodiscard]] bool simulationActive() const noexcept { return simulation_active_; }
    [[nodiscard]] std::vector<Cell> snapshot() {
        initializeCachedFrame();
        Frame frame = cached_original_frame_.clone();
        stampAllDynamicObjectsOntoFrame_(frame);
        return bmpToCells(utils::matToArrayVector(frame));
    }
    [[nodiscard]] int curentFrameIndex() const {return cap_frame_head_ + cap_frame_size_;}
    [[nodiscard]] Frame currentFrame() {
        if (capture_frames_.empty()) {
            logger().warn("No frames captured yet. Returning the original map frame.");
            return cached_original_frame_;
        }
        return capture_frames_[cap_frame_head_ + cap_frame_size_];
    }
    /** Return a raw pointer to the cell at (x,y).  *No bounds check*. */
    Cell* cellPtr(int x, int y) { return &map_data_[idx(x,y)]; }

    // Cells utilities
    /**
     * @brief Converts a BGR color array to a corresponding Cell type
     *
     * @param color An array of 3 uint8_t values representing BGR color channels
     * @return Cell The corresponding Cell enum value for that color
     */
    static Cell colorToCell(std::array<std::uint8_t, 3> color) { return ::colorToCell(color); }

    /**
     * @brief Converts a Cell type to its corresponding BGR color
     *
     * @param cell The Cell enum value to convert
     * @return std::array<std::uint8_t, 3> BGR color array corresponding to the cell type
     */
    static std::array<std::uint8_t, 3> cellToColor(Cell cell) { return ::cellToColor(cell); }

    /**
     * @brief Converts a Cell type to a string representation
     *
     * @param cell The Cell enum value to convert
     * @return std::string String representation of the cell type (e.g. "Unknown", "Free", etc.)
     */
    static std::string cellToString(Cell cell) { return ::cellToString(cell); }

    /**
     * @brief Convers the string representation of a cell type to the corresponding Cell enum value.
     *
     * @param cell_name The string representation of the cell type (e.g. "Unknown", "Free", etc.)
     * @return Cell The corresponding Cell enum value
     */
    static Cell stringToCell(const std::string& cell_name) { return ::stringToCell(cell_name); }

    /**
     * @brief Loads the cells colors from a json file.
     *
     * @param path The path to the json file. The colors have to be in the format:
     * ```json
     * {
     *   "Unknown": [0, 0, 0],
     *   "Free": [255, 255, 255],
     *   ...
     *   "Reserved": [255, 0, 0]
     *   }
     */
    static void loadCellColors(const std::filesystem::path &path) { ::loadCellColors(path); }

    /**
     * @brief Converts a BMP image matrix to a vector of Cell objects.
     *
     * @param bmp_matrix A vector of arrays representing the BMP image, where each array contains 3 uint8_t values (BGR color channels).
     *
     * @return std::vector<Cell> A vector of Cell objects corresponding to the colors in the BMP image.
     */
    static std::vector<Cell> bmpToCells(const std::vector<std::array<uint8_t, 3>>& bmp_matrix);

    /**
     * @brief converts the map to a bmp representation.
     */
    [[nodiscard]] std::vector<std::array<uint8_t, 3>> toBmp() const;

    /**
     * @brief converts the cell data into a bmp representation.
     */
    [[nodiscard]] static std::vector<std::array<uint8_t, 3>> toBmp(std::vector<Cell> cell_data);


    /**
     * @brief Registers the Object on this map. If the object is
     * static, it will be rasterized into the map data immediately.
     *
     * @note call `std::move(object)` to add the object. Passing the
     * pointer directly won't work.
     *
     * @param object unique_ptr to object to be registered
     */
    void addObject(std::unique_ptr<MapObject> object);

    /**
     * @brief Start all objects in the map by calling their start() method
     */
    void startAllObjects() const;

    /** Start buffering frames of the current map view in a separate thread.
     * When this method is called, it updates the motion of the objects.
     *
     * If the object is not started yet, it will be frozen immediately by
     * setting its velocity to zero.
     *
     * @detail The simulation uses a buffer that grows by 10% of the
     * max frame count when the buffer is full. If the max frame count
     * is reached, the video frames after that will not be stored.
     *
     * @note This method returns immediately; the simulation runs in a separate thread.
     * @note If the max. frame count is reached, the simulation will stop automatically.
     * @note If the simulation is already running, the current simulation will be stopped
     *      and a new one will be started.
     */
    void startSimulation();

    /** Terminates the simulation thread */
    void endSimulation();

    /** Flush the buffer to an .mp4 file and reset the capture
     *
     * @param filename The name of the output video file
     */
    void flushFrames(const std::string& filename);

    /** Sets max. number of frames to store in the buffer.
     * If the buffer is full, the oldest frames will be removed.
     *
     * @param max_frames The maximum number of frames to store in the buffer
     */
    void setMaxFramesStored(unsigned int max_frames) {
        max_frames_stored_ = max_frames;
    }

    /**
     * @brief Sets the map resolution in meters/pixel.
     */
    void setResolution(float resolution) {
        if (resolution <= 0) {
            throw std::invalid_argument("Resolution must be positive.");
        }
        resolution_ = resolution;
        invalidateFrameCache();
    }

    /** Query helper                                            */
    [[nodiscard]] bool isSimulating() const { return simulation_active_; }

    /** Return a BGR snapshot of the current grid (1 pixel per cell) */
    [[nodiscard]] Frame renderToMat() const;

    /** Push a user-prepared frame into the buffer. Overloads the *addMotionFrame()* method to allow adding frames that are not
     * already rendered from the map data.
     *
     * @param frame The frame to add to the buffer. It should be a cv::Mat object with the same size as the map.
     */
    void addMotionFrame(const Frame &frame);

    /**
     * @brief Returns the pixel coordinates based on the world coordinates.
     *
     * @note world_x / resolution_ is the raw pixel coordinate. Casting to int will round it down.
     * But, if the raw value goes into the next pixel (e.g. for resolution 0.2, and the pixel value is
     * 38.9), it actually belongs to the next pixel (39). So we need to add the resolution to ensure
     * that we round up correctly.
     *
     * @param world_x The x coordinate in world coordinates
     * @param world_y The y coordinate in world coordinates
     * @return std::pair<int, int> The pixel coordinates (x, y) in the map
     */
    [[nodiscard]] std::pair<int, int> worldToCell(float world_x, float world_y) const;

    /** @return the graph data of the map */
    [[nodiscard]] GraphData& graphData() { return graph_data_; }

    /**
     * @brief Build a road graph from the road objects in the map.
     * This constructs nodes at intersections and endpoints, and edges along roads.
     */
    void buildGraph() { buildGraph_(); }

private:
    size_t cap_frame_head_ = 0; // index of logical front
    size_t cap_frame_size_ = 0; // number of valid frames in buffer
    size_t cap_frame_chunk_ = 0; // size of one growth chunk
    size_t cap_frame_capacity_ = 0; // current allocated capacity

    /**
     * @brief Ensure the ring has capacity for +1 push. Grows by 1/10 of max_frames_stored_ if needed.
     */
    void ensureCaptureCapacityForNextPush_();

    /**
     * @brief Start the global decoder thread that adds video
     * frames continuously while the simulation is active.
     */
    void startFrameDecoderThread_();

    /**
     * @brief Stop the global decoder thread.
     */
    void stopDecoder_();

    /**
     * @brief starts updating the physics of all objects in the map.
     */
    void startUpdatingObjectsPhysics_();
    /**
     * @brief stops updating the physics. Should only be called
     * when the simulation is ending.
     */
    void stopUpdatingObjectsPhysics_();


    [[nodiscard]] int idx(int x, int y) const noexcept { return y * width_ + x; }
    [[nodiscard]] std::pair<int, int> idxToPx(int idx) const noexcept {
        return {idx % width_, idx / width_};
    }
    int width_ = MAX_MAP_WIDTH;  // [pixels]
    int height_ = MAX_MAP_HEIGHT;  // [pixels]
    float resolution_ = DEFAULT_MAP_RESOLUTION; // meters/pixel
    int origin_x_ = 0; // origin x coordinate in pixels
    int origin_y_ = 0; // origin y coordinate in pixels
    std::vector<Cell> map_data_;
    std::vector<std::unique_ptr<MapObject>> objects_;    ///< dynamic objects attached to this map
    MapObject* background_free_object_ = nullptr; ///< The object that occupies the whole map upon creation
    [[nodiscard]] static spdlog::logger& logger() {
        static std::shared_ptr<spdlog::logger> logger_ = utils::getLogger("Map2D");
        return *logger_;
    }
    GraphData graph_data_;

    /**
     * @brief tracks how long each object has been off-map. After MAX_OFFMAX_TIME seconds,
     * the object will be destroyed and de-registered from the map.
     */
    std::unordered_map<MapObject*, std::chrono::steady_clock::time_point> offmap_time_;
    bool simulation_active_{false};
    std::vector<Frame> capture_frames_;
    size_t max_frames_stored_{MAX_FRAMES_STORED};

    // Cache for the original frame without dynamic objects
    mutable Frame cached_original_frame_;
    mutable bool frame_cache_valid_{false};

    /** Initialize or refresh the cached original frame */
    void initializeCachedFrame() const;

    /**
     * Invalidate the cached frame (call when static map data changes)
     * @note also clears the snapshot.
     */
    void invalidateFrameCache() const;

    /**
     * @brief Rasterize a (convex) static object polygon directly into map_data_ with its
     * Cell type and invalidate the cached frame.
     * @note This method overwrites the cached_original_frame_ directly!
     */
    void stampStaticObject_(const std::unique_ptr<MapObject>& object);

    /**
     * @brief stamps all dynamic objects onto the frame.
     *
     * @note this method was made to avoid stamping static objects onto the frame
     * while simulating because static objects never move anyway.
     */
    void stampAllDynamicObjectsOntoFrame_(Frame& frame);

    /**
     * @brief Remove object from this map and destroy its Box2D body
     *
     * @param obj The object to remove
     */
    void removeObject_(std::unique_ptr<MapObject> obj);

    /**
     * @brief Return true if the object's polygon intersects the image
     * frame.
     *
     * @note Used a raw pointer here to avoid ownership issues, because
     * this method does not modify the object anyway.
     *
     * @param obj The object to check
     */
    bool objectIsInMap_(MapObject* obj) const;

    /**
     * @brief Check if an object has been off-map for too long.
     *
     * Policy:
     * - If fully off-frame, start/continue TTL.
     * - If off-frame duration >= OFFMAP_TTL_SECONDS, request removal.
     * - If back on-frame, clear TTL.
     *
     * @note Used a raw pointer here to avoid ownership issues, because
     * this method does not modify the object anyway.
     *
     * @param obj Dynamic object pointer (non-null for evaluation).
     *
     * @return bool True if the object has been off-map for too long and should be removed.
     */
    bool objectOffMapTooLong_(MapObject* obj);

    /**
     * @brief removes all dynamic objects from the map.
     */
    void destroyAllDynamicObjects();

    /**
     * @brief Capture a frame by drawing the static map, then overlaying
     *        each dynamic object's polygon filled in its BGR color.
     *
     * @details this method only stamps the **dynamic objects** on top of a copy of
     * the static map.
     * @details Does NOT mutate map_data_; it uses the physics state for positions.
     */
    void addMotionFrame();

    /**
     * @brief Fills the entire map data with the cell type.
     *
     * @param cell The cell type
     */
    void fillMapWith(Cell cell);

    /**
     * @brief Stamps the given object directly onto the given frame and
     * overwrite the current map cell data.
     *
     * @param frame The frame to stamp objects onto.
     */
    void stampObjectOntoFrame_(const std::unique_ptr<MapObject>& obj, Frame& frame);

    /**
     * @brief Rebuild road graph
     */
    void buildGraph_();

    /**
     * @brief Clears the graph and node coordinate containers.
     *
     */
    void resetGraphStorage_();

    /**
     * @brief Collect road objects from all objects on the map
     * in order to build the road graph.
     */
    std::vector<Road*> collectRoads_() const;

    /**
     * Add intersection nodes by iterating over all pairs of
     * road objects and checking for intersection.
     *
     */
    void addIntersectionNodes_(
        const std::vector<Road*>& roads,
        std::unordered_map<Road*, std::vector<int>>& roadNodes);

    /** Ensure endpoints exist per road. */
    void addEndpointNodes_(
        const std::vector<Road*>& roads,
        std::unordered_map<Road*, std::vector<int>>& roadNodes);

    /** Connect consecutive nodes along each road. */
    void addEdgesAlongRoads_(
        const std::vector<Road*>& roads,
        const std::unordered_map<Road*, std::vector<int>>& roadNodes);

    /**
     * @return the 4 world corners ot a road's oriented rectangle (OBB) in meters.
     */
    std::array<b2Vec2,4> roadCorners_(const Road* r) const;

    /**
    * @return true if the two oriented bounding boxes overlap (2D SAT test).
    */
    static bool obbOverlap_(const std::array<b2Vec2,4>& a, const std::array<b2Vec2,4>& b) ;

    /**
     * @brief Checks if the centerlines of two road segments intersect within
     * each segment's length. If yes, return that point in p.
     *
     * @detail fpr each road, compute the centers C1 and C2, direction
     * of unit vector d1 and d2, half lengths L1 and L2, solve the 2D line system,
     * and check whether the parameters t and u are within [-L1, L1] and [-L2, L2].
     *
     * @param r1 First road segment
     * @param r2 Second road segment
     * @param p Output intersection point if return value is true
     * @return
     */
    static bool centerlineCrossPoint_(const Road* r1, const Road* r2, b2Vec2& p);

    /**
     * Add a node to the global graph at the given world coordinates.
     *
     * @param p: world coordinates in meters
     * @return the id of the newly inserted node
     */
    int  addNode_(const b2Vec2& p);
};

/**
 * @brief Overloads the << operartor to allow printing the Cell enum to an output stream as a string.
 *
 * @param os The output stream
 * @param cell The Cell enum to print
 * @return std::ostream& The output stream
 */
std::ostream& operator<<(std::ostream& os, Cell cell);

/**
 * @brief Returns the global map instance. Subsequent calls will return the same instance.
 *
 * @return Map2D& A reference to the global map.
 */
Map2D& globalMap();

#endif //MAP2D_H
