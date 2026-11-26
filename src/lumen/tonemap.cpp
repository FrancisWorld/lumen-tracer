#include "lumen/tonemap.hpp"

#include <stdexcept>

#include "lumen/texture.hpp"

namespace lumen {

Vec3 aces_tonemap(const Vec3& c) {
    auto curve = [](Real x) {
        x = std::max(x, Real(0));
        return clamp01((x * (Real(2.51) * x + Real(0.03))) / (x * (Real(2.43) * x + Real(0.59)) + Real(0.14)));
    };
    return {curve(c.x), curve(c.y), curve(c.z)};
}

Image8 to_srgb8(const std::vector<Vec3>& hdr, std::uint32_t width, std::uint32_t height, Real exposure) {
    if (hdr.size() != static_cast<std::size_t>(width) * height) {
        throw std::invalid_argument("to_srgb8: buffer size does not match dimensions");
    }
    Image8 img{width, height, 3, Bytes(hdr.size() * 3)};
    for (std::size_t i = 0; i < hdr.size(); ++i) {
        const Vec3 mapped = aces_tonemap(hdr[i] * exposure);
        for (int k = 0; k < 3; ++k) {
            img.pixels[i * 3 + static_cast<std::size_t>(k)] =
                static_cast<std::uint8_t>(std::lround(linear_to_srgb(mapped[k]) * Real(255)));
        }
    }
    return img;
}

}  // namespace lumen
