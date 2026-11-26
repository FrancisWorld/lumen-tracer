// PNG encoder and decoder written from the specification. Supports 8-bit
// grayscale, RGB, grayscale+alpha and RGBA, non-interlaced: all the renderer
// produces and all it needs for texture input.
#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <string>

#include "lumen/deflate.hpp"

namespace lumen {

struct Image8 {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t channels = 3;  // 1 = gray, 2 = gray+alpha, 3 = RGB, 4 = RGBA
    Bytes pixels;                // row-major, tightly packed
};

/// Throws std::invalid_argument when the buffer size does not match the dimensions.
Bytes encode_png(const Image8& image);
std::expected<Image8, std::string> decode_png(std::span<const std::uint8_t> data);

void write_png_file(const std::string& path, const Image8& image);
std::expected<Image8, std::string> read_png_file(const std::string& path);

}  // namespace lumen
