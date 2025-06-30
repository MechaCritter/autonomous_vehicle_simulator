//
// Created by vunha on 4/20/2025.
//

#include "utils.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <filesystem>

// private methods used only in this file. NOTE: the color scheme is BGR
namespace {
// helper to write little‑endian word
template<typename T> void le32(std::ofstream& out, T v) {
    out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}
}

std::vector<std::array<uint8_t,3>> utils::loadBmp(const std::string& p, int& w, int& h) {
    std::ifstream in(p, std::ios::binary);
    std::cout << "Loading map from: " << p << std::endl;
    if (!in) throw std::runtime_error("bmp open failed");

    uint16_t bfType; in.read(reinterpret_cast<char*>(&bfType),2);
    if (bfType != 0x4D42) throw std::runtime_error("not BMP");

    in.seekg(18);
    in.read(reinterpret_cast<char*>(&w),4); // These lines assign the image's size to w and h (w and h were not initialized with a value)
    in.read(reinterpret_cast<char*>(&h),4);
    in.seekg(2, std::ios::cur);               // skip planes
    uint16_t bpp; in.read(reinterpret_cast<char*>(&bpp),2);
    if (bpp != 24) throw std::runtime_error("need 24‑bit BMP");

    uint32_t dataOffset; in.seekg(10); in.read(reinterpret_cast<char*>(&dataOffset),4);
    in.seekg(dataOffset);

    std::vector<std::array<uint8_t,3>> out(w*h);
    int pad = (4 - (w*3)%4)%4;
    for (int y = h-1; y >= 0; --y) {          // BMP stored bottom‑up
        for (int x = 0; x < w; ++x) {
            uint8_t bgr[3]; in.read(reinterpret_cast<char*>(bgr),3);
            out[y*w+x] = { bgr[0], bgr[1], bgr[2] };
        }
        in.seekg(pad, std::ios::cur);
    }
    return out;
}

void utils::writeBmp(const std::string& path,
                     int width,int height,
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
    le32(out,int32_t(width));
    le32(out,int32_t(height));
    out.put(1); out.put(0);                 // planes
    out.put(24); out.put(0);                // bpp = 24
    le32(out,uint32_t{0});                  // compression
    le32(out,rowBytes*height);              // image size
    le32(out,int32_t{2835});                // x ppm
    le32(out,int32_t{2835});                // y ppm
    le32(out,uint32_t{0});                  // palette
    le32(out,uint32_t{0});                  // important cols

    const uint8_t pad[3] = {0,0,0};
    for (int y = 0; y < height; ++y) {
        int row = topDown ? y : height-1-y;
        for (int x = 0; x < width; ++x) {
            const auto& c = bgr[row*width+x];
            out.write(reinterpret_cast<const char*>(c.data()),3);
        }
        out.write(reinterpret_cast<const char*>(pad), (4-(width*3)%4)%4);
    }
}





