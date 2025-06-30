#ifndef MAP2D_H
#define MAP2D_H
#include <vector>
#include <format>
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include <spdlog/spdlog.h>

#include "../data/classes.h"

constexpr int MAX_MAP_WIDTH = 1000;
constexpr int MAX_MAP_HEIGHT = 1000;
constexpr double DEFAULT_MAP_RESOLUTION = 0.1; // meters/pixel
// const std::string DEFAULT_MAP_FILE = "../../res/maps/default_map.bmp"; // TODO: find a way to make relative path to work again
const std::string DEFAULT_MAP_FILE = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/default_map.bmp";

class Map2D {
public:
    /**
     * @brief Construct a new Map2D object. This object holds a pixmap where each
     * pixel represents a cell in the map. Each cell represents a class.
     *
     */
    Map2D(
        int width,
        int height,
        double res_m,
        Cell default_value = Cell::Unknown
        );
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

private:
    static spdlog::logger &logger(); // one logger obj shared across all Map2D instances
    [[nodiscard]] int idx(int x, int y) const noexcept { return y * width_ + x; }
    int width_ = MAX_MAP_WIDTH;
    int height_ = MAX_MAP_HEIGHT;
    double resolution_ = DEFAULT_MAP_RESOLUTION; // meters/pixel
    int origin_x_ = 0; // origin x coordinate in pixels
    int origin_y_ = 0; // origin y coordinate in pixels
    std::vector<Cell> map_data_;
    // A hash table that maps each cell type to its corresponding BGR color.
    static std::unordered_map<Cell, std::array<std::uint8_t, 3>> cell_colors_;

#ifdef WITH_OPENCV_DEBUG                               // ← add begin
public:
    /** Start buffering frames of the current map view         */
    void startMotionCapture();

    /** Add one frame (current map state) to the buffer         */
    void addMotionFrame();

    /** Stop buffering frames of the current map view */
    void endMotionCapture();

    /** Flush the buffer to an .mp4 file and reset the capture
     *
     * @param filename The name of the output video file
     * @param fps The frames per second of the output video
     */
    void flushMotionCapture(const std::string& filename,
                          double fps = 15.0);

    /** Query helper                                            */
    [[nodiscard]] bool isCapturing() const { return capture_active_; }

    /** Return a BGR snapshot of the current grid (1 pixel per cell) */
    cv::Mat renderToMat() const;

    /** Push a user-prepared frame into the buffer. Overloads the *addMotionFrame()* method to allow adding frames that are not
     * already rendered from the map data.
     *
     * @param frame The frame to add to the buffer. It should be a cv::Mat object with the same size as the map.
     */
    void addMotionFrame(const cv::Mat& frame);

private:
    bool capture_active_{false};
    std::vector<cv::Mat> capture_frames_;
#endif
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
