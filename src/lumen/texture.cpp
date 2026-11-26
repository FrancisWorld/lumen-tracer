#include "lumen/texture.hpp"

#include <cmath>

namespace lumen {

Real srgb_to_linear(Real v) {
    return v <= Real(0.04045) ? v / Real(12.92) : std::pow((v + Real(0.055)) / Real(1.055), Real(2.4));
}

Real linear_to_srgb(Real v) {
    v = clamp01(v);
    return v <= Real(0.0031308) ? v * Real(12.92) : Real(1.055) * std::pow(v, Real(1) / Real(2.4)) - Real(0.055);
}

ImageTexture ImageTexture::from_image(const Image8& image, bool srgb, Vec2 repeat) {
    ImageTexture t;
    t.width = image.width;
    t.height = image.height;
    t.repeat = repeat;
    t.texels.resize(static_cast<std::size_t>(image.width) * image.height);
    for (std::size_t i = 0; i < t.texels.size(); ++i) {
        Real c[3];
        for (std::uint32_t k = 0; k < 3; ++k) {
            // Gray and gray+alpha images replicate channel 0; colour images use their own channels.
            const std::uint32_t ch = image.channels >= 3 ? k : 0;
            const Real v = static_cast<Real>(image.pixels[i * image.channels + ch]) / Real(255);
            c[k] = srgb ? srgb_to_linear(v) : v;
        }
        t.texels[i] = {c[0], c[1], c[2]};
    }
    return t;
}

namespace {

Vec3 sample_image(const ImageTexture& t, Vec2 uv) {
    const Real u = uv.x * t.repeat.x;
    const Real v = uv.y * t.repeat.y;
    const Real fx = (u - std::floor(u)) * static_cast<Real>(t.width) - Real(0.5);
    const Real fy = ((Real(1) - (v - std::floor(v))) * static_cast<Real>(t.height)) - Real(0.5);  // flip: row 0 is the top
    const Real x0f = std::floor(fx);
    const Real y0f = std::floor(fy);
    const Real tx = fx - x0f;
    const Real ty = fy - y0f;
    auto wrap = [](long i, std::uint32_t n) {
        const long m = static_cast<long>(n);
        return static_cast<std::size_t>(((i % m) + m) % m);
    };
    const long x0 = static_cast<long>(x0f);
    const long y0 = static_cast<long>(y0f);
    auto at = [&](long x, long y) { return t.texels[wrap(y, t.height) * t.width + wrap(x, t.width)]; };
    const Vec3 top = lerp(at(x0, y0), at(x0 + 1, y0), tx);
    const Vec3 bottom = lerp(at(x0, y0 + 1), at(x0 + 1, y0 + 1), tx);
    return lerp(top, bottom, ty);
}

}  // namespace

Vec3 evaluate(const Texture& texture, Vec2 uv) {
    struct Visitor {
        Vec2 uv;
        Vec3 operator()(const ConstantTexture& c) const { return c.color; }
        Vec3 operator()(const CheckerTexture& c) const {
            const long iu = static_cast<long>(std::floor(uv.x * c.scale * 2));
            const long iv = static_cast<long>(std::floor(uv.y * c.scale * 2));
            return ((iu + iv) & 1L) == 0 ? c.a : c.b;
        }
        Vec3 operator()(const ImageTexture& t) const { return sample_image(t, uv); }
    };
    return std::visit(Visitor{uv}, texture);
}

}  // namespace lumen
