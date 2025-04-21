#ifndef MAP2D_H
#define MAP2D_H
#include <vector>
#include <format>
#include <string>

#include "../data/classes.h"

constexpr int MAX_MAP_WIDTH = 1000;
constexpr int MAX_MAP_HEIGHT = 1000;
constexpr double DEFAULT_MAP_RESOLUTION = 0.1; // meters/pixel
const std::string DEFAULT_MAP_FILE = "../../res/maps/default_map.bmp";

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
    [[nodiscard]] double resolution() const noexcept {return res_m_;}
private:
    [[nodiscard]] int idx(int x, int y) const noexcept { return y * width_ + x; }
    int width_ = MAX_MAP_WIDTH;
    int height_ = MAX_MAP_HEIGHT;
    double res_m_ = 0.1; // resolution (meters/pixel)
    int origin_x_ = 0; // origin x coordinate in pixels
    int origin_y_ = 0; // origin y coordinate in pixels
    std::vector<Cell> map_;
};

/**
 * @brief Returns the global map instance. Subsequent calls will return the same instance.
 *
 * @return const Map2D& A constant reference to the global map.
 */
const Map2D& globalMap();


#endif //MAP2D_H
