#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <array>
#include <vector>
#include <stdexcept>

namespace {
    // helper to write little‑endian word
    template<typename T> void le32(std::ofstream& out, T v) {
        out.write(reinterpret_cast<const char*>(&v), sizeof(T));
    }
}

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
    inline std::vector<std::array<uint8_t,3>> loadBmp(const std::string& path, int& width, int& height) {
        std::ifstream in(path, std::ios::binary);
        std::cout << "Loading map from: " << path << std::endl;
        if (!in) throw std::runtime_error("bmp open failed");

        uint16_t bfType; in.read(reinterpret_cast<char*>(&bfType),2);
        if (bfType != 0x4D42) throw std::runtime_error("not BMP");

        in.seekg(18);
        in.read(reinterpret_cast<char*>(&width),4); // These lines assign the image's size to w and h (w and h were not initialized with a value)
        in.read(reinterpret_cast<char*>(&height),4);
        in.seekg(2, std::ios::cur);               // skip planes
        uint16_t bpp; in.read(reinterpret_cast<char*>(&bpp),2);
        if (bpp != 24) throw std::runtime_error("need 24‑bit BMP");

        uint32_t dataOffset; in.seekg(10); in.read(reinterpret_cast<char*>(&dataOffset),4);
        in.seekg(dataOffset);

        std::vector<std::array<uint8_t,3>> out(width*height);
        int pad = (4 - (width*3)%4)%4;
        for (int y = height-1; y >= 0; --y) {          // BMP stored bottom‑up
            for (int x = 0; x < width; ++x) {
                uint8_t bgr[3]; in.read(reinterpret_cast<char*>(bgr),3);
                out[y*width+x] = { bgr[0], bgr[1], bgr[2] };
            }
            in.seekg(pad, std::ios::cur);
        }
        return out;
    }
    /**
     * @brief Saves an image to a .bmp file
     *
     * @param path Full path to the output BMP file
     * @param width Width of the image in pixels
     * @param height Height of the image in pixels
     * @param bgr Vector containing BGR color values for each pixel, ordered from top-left to bottom-right
     * @param topDown If true, the image is saved with the top row at the top of the file
     */
    inline void writeBmp(const std::string& path,
                         const int width,
                         const int height,
                         const std::vector<std::array<uint8_t,3>>& bgr,
                         bool topDown)
    {
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());

        const uint32_t rowBytes = width*3 + (4-(width*3)%4)%4;
        const uint32_t fileSize = 54 + rowBytes*height;

        std::ofstream out(path, std::ios::binary);
        out.write("BM",2);                      // signature
        le32(out,fileSize);                     // file size
        le32(out,uint32_t{0});                  // reserved
        le32(out,uint32_t{54});                 // pixel data offset
        le32(out,uint32_t{40});                 // DIB header size
        le32(out,width);
        le32(out,height);
        out.put(1); out.put(0);                 // planes
        out.put(24); out.put(0);                // bpp = 24
        le32(out,uint32_t{0});                  // compression
        le32(out,rowBytes*height);              // image size
        le32(out,int32_t{2835});                // x ppm
        le32(out,int32_t{2835});                // y ppm
        le32(out,uint32_t{0});                  // palette
        le32(out,uint32_t{0});                  // important cols

        for (int y = 0; y < height; ++y) {
            const uint8_t pad[3] = {0,0,0};
            int row = topDown ? y : height-1-y;
            for (int x = 0; x < width; ++x) {
                const auto& c = bgr[row*width+x];
                out.write(reinterpret_cast<const char*>(c.data()),3);
            }
            out.write(reinterpret_cast<const char*>(pad), (4-(width*3)%4)%4);
        }
    }

    /**
     * @brief Convert an OpenCV Mat (CV_8UC3) into a vector of BGR triplets.
     *
     * @param frame Input image (must be CV_8UC3).
     * @return std::vector<std::array<uint8_t, 3>> Flattened pixel values in BGR order.
     *
     * @throws std::invalid_argument if frame is empty or not CV_8UC3.
     */
    inline std::vector<std::array<uint8_t, 3>> matToArrayVector(const cv::Mat& frame) {
        if (frame.empty()) {
            throw std::invalid_argument("Input frame is empty.");
        }
        if (frame.type() != CV_8UC3) {
            throw std::invalid_argument("Input frame must be CV_8UC3 (8-bit, 3-channel).");
        }

        std::vector<std::array<uint8_t, 3>> pixels;
        pixels.reserve(frame.rows * frame.cols);

        for (int r = 0; r < frame.rows; ++r) {
            const auto* row_ptr = frame.ptr<cv::Vec3b>(r);
            for (int c = 0; c < frame.cols; ++c) {
                const cv::Vec3b& px = row_ptr[c]; // BGR pixel
                pixels.push_back({px[0], px[1], px[2]});
            }
        }

        return pixels;
    }
}