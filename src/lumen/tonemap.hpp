#pragma once

#include <vector>

#include "lumen/math.hpp"
#include "lumen/png.hpp"

namespace lumen {

/// Narkowicz's fit of the ACES filmic curve, applied per channel; output in [0, 1], linear.
Vec3 aces_tonemap(const Vec3& linear);

/// Exposure, ACES, sRGB transfer and 8-bit quantisation into an RGB image.
Image8 to_srgb8(const std::vector<Vec3>& hdr, std::uint32_t width, std::uint32_t height, Real exposure);

}  // namespace lumen
