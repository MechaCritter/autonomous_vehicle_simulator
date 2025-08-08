#ifndef MAP2D_H
#define MAP2D_H
#include <vector>
#include <format>
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include <unordered_map>

#include "../data/classes.h"
#include "../utils/utils.h"
#include "../utils/logging.h"
#include "../base/MapObject.h"

constexpr int MAX_MAP_WIDTH = 1000;
constexpr int MAX_MAP_HEIGHT = 1000;
constexpr double DEFAULT_MAP_RESOLUTION = 0.1; // meters/pixel
// const std::string DEFAULT_MAP_FILE = "../../res/maps/default_map.bmp"; // TODO: find a way to make relative path to work again
const std::string DEFAULT_MAP_FILE = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/default_map.bmp";

class Vehicle; // forward declaration, otherwise circular dependency with Vehicle.h

class Map2D {
public:
    /**
     * @brief Construct a new Map2D object. This object holds a pixmap where each
     * pixel represents a cell in the map. Each cell represents a class.
     *
     * @param width Width of the map in pixels
     * @param height Height of the map in pixels
     * @param res_m Resolution of the map in meters per pixel
     */
    Map2D(
    int width,
    int height,
    double res_m
    );
    /**
     * @brief Construct a new Map2D object. This object holds a pixmap where each
     * pixel represents a cell in the map. Each cell represents a class.
     *
     * @param width Width of the map in pixels
     * @param height Height of the map in pixels
     * @param res_m Resolution of the map in meters per pixel
     * @param default_value The default cell value to fill the entire map with.
     */
    Map2D(
        int width,
        int height,
        double res_m,
        Cell default_value
        );
    Map2D(const Map2D&) = delete;          // still non-copyable
    Map2D& operator=(const Map2D&) = delete;

    Map2D(Map2D&&) noexcept = default;     // now movable
    Map2D& operator=(Map2D&&) noexcept = default;

    /**
     * @brief Destructor.
     */
    ~Map2D() = default;

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

    static Map2D loadMap(const std::string& filename, double res_m);

    //getters
    [[nodiscard]] int width() const noexcept {return width_;}
    [[nodiscard]] int height() const noexcept {return height_;}
    [[nodiscard]] double resolution() const noexcept {return resolution_;}
    [[nodiscard]] static const std::unordered_map<Cell, std::array<std::uint8_t, 3>>& cell_colors() noexcept
                        {return cell_colors_;}
    [[nodiscard]] const std::vector<MapObject*>& objects() const noexcept { return objects_; }
    [[nodiscard]] int maxFrameStored() const noexcept { return max_frames_stored_; }
    [[nodiscard]] bool simulationActive() const noexcept { return simulation_active_; }
    /** Return a raw pointer to the cell at (x,y).  *No bounds check*. */
    Cell* cellPtr(int x, int y) { return &map_data_[idx(x,y)]; }

    // Cells utilities
    /**
     * @brief Converts a BGR color array to a corresponding Cell type
     *
     * @param color An array of 3 uint8_t values representing BGR color channels
     * @return Cell The corresponding Cell enum value for that color
     */
    static Cell colorToCell(std::array<std::uint8_t, 3> color);

    /**
     * @brief Converts a Cell type to its corresponding BGR color
     *
     * @param cell The Cell enum value to convert
     * @return std::array<std::uint8_t, 3> BGR color array corresponding to the cell type
     */
    static std::array<std::uint8_t, 3> cellToColor(Cell cell);

    /**
     * @brief Converts a Cell type to a string representation
     *
     * @param cell The Cell enum value to convert
     * @return std::string String representation of the cell type (e.g. "Unknown", "Free", etc.)
     */
    static std::string cellToString(Cell cell);

    /**
     * @brief Convers the string representation of a cell type to the corresponding Cell enum value.
     *
     * @param cell_name The string representation of the cell type (e.g. "Unknown", "Free", etc.)
     * @return Cell The corresponding Cell enum value
     */
    static Cell stringToCell(const std::string& cell_name);;

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
    static void loadCellColors(const std::filesystem::path &path);

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
     * @brief Fills the entire map data with the cell type.
     *
     * @param cell The cell type
     */
    void fillMapWith(Cell cell);

    /**
     * @brief Registers the Object on this map
     *
     * @param object object to be registered
     */
    void addObject(MapObject& object);

    /**
     * @brief Removes all the footprints of the objects on the map and
     * restores the previous cell values.
     */
     void removeFootprints() const;

    /** Start buffering frames of the current map view in a separate thread.
     * When this method is called, it updates the motion of the objects on
     * it until the *endSimulation()* method is called, or the max. frame count
     * is reached.
     */
    void startSimulation();

    /**
     * @brief Starts the simulation for a specified duration in seconds.
     * The simulation will run and update the map and objects for the given time,
     * then automatically stop.
     *
     * @param seconds Duration to run the simulation, in seconds.
     */
    void startSimulationFor(unsigned int seconds);

    /** Add one frame (current map state) to the buffer         */
    void addMotionFrame();

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
    void setMaxFramesStored(unsigned int max_frames) {max_frames_stored_ = max_frames;}

    /** Query helper                                            */
    [[nodiscard]] bool isSimulating() const { return simulation_active_; }

    /** Return a BGR snapshot of the current grid (1 pixel per cell) */
    [[nodiscard]] cv::Mat renderToMat() const;

    /** Push a user-prepared frame into the buffer. Overloads the *addMotionFrame()* method to allow adding frames that are not
     * already rendered from the map data.
     *
     * @param frame The frame to add to the buffer. It should be a cv::Mat object with the same size as the map.
     */
    void addMotionFrame(const cv::Mat& frame);

    // /**
    //  *
    //  * @param Rasterise a world-frame rectangle (vehicle) into occupied cells.
    //  *
    //  * @param vehicle The vehicle to stamp on the map
    //  */
    // void stampVehicle(const Vehicle& vehicle);

    /**
     *
     * @brief Returns the pixel coordinates based on the world coordinates.
     * **NOTE**: world_x / resolution_ is the raw pixel coordinate. Casting to int will round it down.
     * But, if the raw value goes into the next pixel (e.g. for resolution 0.2, and the pixel value is
     * 38.9), it actually belongs to the next pixel (39). So we need to add the resolution to ensure
     * that we round up correctly.
     *
     * @param world_x The x coordinate in world coordinates
     * @param world_y The y coordinate in world coordinates
     * @return std::pair<int, int> The pixel coordinates (x, y) in the map
     */
    [[nodiscard]] std::pair<int, int> worldToCell(float world_x, float world_y) const;

private:
    [[nodiscard]] int idx(int x, int y) const noexcept { return y * width_ + x; }
    int width_ = MAX_MAP_WIDTH;
    int height_ = MAX_MAP_HEIGHT;
    double resolution_ = DEFAULT_MAP_RESOLUTION; // meters/pixel
    int origin_x_ = 0; // origin x coordinate in pixels
    int origin_y_ = 0; // origin y coordinate in pixels
    std::vector<Cell> map_data_;
    std::vector<Cell> base_data_;        ///< immutable background (roads, obstacles …)
    std::vector<MapObject*> objects_;    ///< dynamic objects attached to this map
    // A hash table that maps each cell type to its corresponding BGR color.
    static std::unordered_map<Cell, std::array<std::uint8_t, 3>> cell_colors_;

    [[nodiscard]] static spdlog::logger& logger() {
        static std::shared_ptr<spdlog::logger> logger_ = utils::getLogger("Map2D");
        return *logger_;
    }
    std::thread simulation_thread_;
    bool simulation_active_{false};
    std::vector<cv::Mat> capture_frames_;
    unsigned int max_frames_stored_{100000};
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
