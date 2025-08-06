//
// Created by vunha on 4/20/2025.
//

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <array>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../data/classes.h"

class MapObject;

namespace utils {
    /**
     * @brief Loads a .bmp image.
     *
     * @param path : Path to the BMP file
     * @param width : Width of the image
     * @param height : Height of the image
     * @return std::vector<Cell> : A vector of Cell objects representing the map
     */
    std::vector<std::array<uint8_t,3>> loadBmp(const std::string& path, int& width, int& height);

    /**
     * @brief Saves an image to a .bmp file
     *
     * @param path Full path to the output BMP file
     * @param width Width of the image in pixels
     * @param height Height of the image in pixels
     * @param bgr Vector containing BGR color values for each pixel, ordered from top-left to bottom-right
     * @param topDown If true, the image is saved with the top row at the top of the file
     */
    void writeBmp(const std::string &path,
                  int width,
                  int height,
                  const std::vector<std::array<uint8_t, 3> > &bgr,
                  bool topDown = false);

    /**
     * @brief Check if a grid-cell centre lies inside a rotated rectangle (vehicle).
     *
    - * @param cell_x, cell_y  Grid indices of the cell to test.
    + * @param cell_x  Column index of the cell.
    + * @param cell_y  Row index of the cell.
     * @param resolution      Map resolution [m/px].
     * @param vehicle         Target vehicle (pose, size).
     * @return true if inside (edges inclusive).
     */
    bool pointIsInRotatedRechtangle(int cell_x,
                          int cell_y,
                          double resolution,
                          const MapObject& vehicle);

    /**
     * @brief Rotate a set of 2-D points about the origin.
     *
     * @param[in] local_pts  Points expressed in the local/object frame.
     * @param[in] angle_rad  Rotation angle in **radians** (CCW-positive).
     * @return               Rotated points (same size/order as the input).
     */
    std::vector<Eigen::Vector2d>
    transform(const std::vector<Eigen::Vector2d>& local_pts,
              double angle_rad);

    /**
     * @brief Rotate a set of 2-D points and then translate them into world-frame.
     *
     * @param[in] local_pts  Points expressed in the vehicle/local frame.
     * @param[in] pose       World-frame translation (x, y) and heading (θ).
     * @return               Points in world coordinates.
     */
    std::vector<Eigen::Vector2d>
    transform(const std::vector<Eigen::Vector2d>& local_pts,
              const Pose2D&                       pose);
}



#endif //UTILS_H
