// Colour textures evaluated in linear RGB.
#pragma once

#include <variant>
#include <vector>

#include "lumen/math.hpp"
#include "lumen/png.hpp"

namespace lumen {

struct ConstantTexture {
    Vec3 color;
};

/// Alternating cells; `scale` is the number of cell pairs per unit of uv.
struct CheckerTexture {
    Vec3 a;
    Vec3 b;
    Real scale = 8;
};

/// Bilinear-filtered, repeating image. v = 0 is the bottom row.
struct ImageTexture {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<Vec3> texels;  // linear RGB
    Vec2 repeat{1, 1};

    /// `srgb` selects whether 8-bit values are gamma-encoded colour (true) or raw data (false).
    static ImageTexture from_image(const Image8& image, bool srgb, Vec2 repeat);
};

using Texture = std::variant<ConstantTexture, CheckerTexture, ImageTexture>;

Vec3 evaluate(const Texture& texture, Vec2 uv);

Real srgb_to_linear(Real v);
Real linear_to_srgb(Real v);

}  // namespace lumen
