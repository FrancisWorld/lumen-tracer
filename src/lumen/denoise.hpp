// Edge-avoiding a-trous wavelet denoiser (Dammertz et al. 2010). Filters
// demodulated irradiance (colour / albedo) with weights from the colour,
// normal and depth guides so texture detail and geometric edges survive.
#pragma once

#include <vector>

#include "lumen/renderer.hpp"

namespace lumen {

struct DenoiseSettings {
    int iterations = 5;        // step widths 1, 2, 4, ... 2^(n-1)
    Real sigma_color = 0.45F;  // luminance tolerance, halves every iteration
    Real sigma_normal = 32;    // exponent applied to max(0, n.n')
    Real sigma_depth = 0.08F;  // relative depth tolerance per pixel of step
    unsigned threads = 0;
};

/// Returns a filtered copy of `fb.color`; the framebuffer is not modified.
std::vector<Vec3> denoise(const Framebuffer& fb, const DenoiseSettings& settings);

}  // namespace lumen
