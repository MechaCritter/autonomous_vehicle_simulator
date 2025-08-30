//
// Created by vunha on 4/20/2025.
//

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <array>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <box2d/box2d.h>
#include "../data/DataClasses.h"

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
     * @brief Convert an OpenCV Mat (CV_8UC3) into a vector of BGR triplets.
     *
     * @param frame Input image (must be CV_8UC3).
     * @return std::vector<std::array<uint8_t, 3>> Flattened pixel values in BGR order.
     *
     * @throws std::invalid_argument if frame is empty or not CV_8UC3.
     */
    std::vector<std::array<uint8_t, 3>> matToArrayVector(const cv::Mat& frame);
}


#endif //UTILS_H
