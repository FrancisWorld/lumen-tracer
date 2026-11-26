#include "lumen/denoise.hpp"

#include <array>

#include "lumen/error.hpp"
#include "lumen/scheduler.hpp"

namespace lumen {
namespace {

// 5-tap B3 spline kernel used by the a-trous transform.
constexpr std::array<Real, 3> kKernel = {Real(3) / 8, Real(1) / 4, Real(1) / 16};

}  // namespace

std::vector<Vec3> denoise(const Framebuffer& fb, const DenoiseSettings& s) {
    if (s.iterations < 0 || s.iterations > 8) {
        throw InputError("denoise: iterations must be between 0 and 8");
    }
    const auto w = static_cast<long>(fb.width);
    const auto h = static_cast<long>(fb.height);
    const std::size_t n = fb.color.size();
    if (fb.albedo.size() != n || fb.normal.size() != n || fb.depth.size() != n) {
        throw InputError("denoise: guide buffers do not match the colour buffer");
    }

    // Demodulate so that albedo texture detail is not blurred away.
    std::vector<Vec3> current(n);
    std::vector<Vec3> next(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 a = vmax(fb.albedo[i], Vec3(Real(0.02)));
        current[i] = fb.color[i] / a;
    }

    constexpr long kBandRows = 8;
    const std::size_t bands = static_cast<std::size_t>((h + kBandRows - 1) / kBandRows);

    for (int iter = 0; iter < s.iterations; ++iter) {
        const long step = 1L << iter;
        const Real sigma_c = s.sigma_color / static_cast<Real>(1 << iter);
        const Real depth_tol = s.sigma_depth * static_cast<Real>(step);
        run_work_stealing(bands, s.threads, [&](std::size_t band, unsigned) {
            const long y_begin = static_cast<long>(band) * kBandRows;
            const long y_end = std::min(y_begin + kBandRows, h);
            for (long y = y_begin; y < y_end; ++y) {
                for (long x = 0; x < w; ++x) {
                    const std::size_t ci = static_cast<std::size_t>(y * w + x);
                    const Real lum_c = luminance(current[ci]);
                    const Vec3 nc = fb.normal[ci];
                    const Real dc = fb.depth[ci];
                    Vec3 sum;
                    Real weight_sum = 0;
                    for (long dy = -2; dy <= 2; ++dy) {
                        for (long dx = -2; dx <= 2; ++dx) {
                            const long qx = x + dx * step;
                            const long qy = y + dy * step;
                            if (qx < 0 || qx >= w || qy < 0 || qy >= h) {
                                continue;
                            }
                            const std::size_t qi = static_cast<std::size_t>(qy * w + qx);
                            Real weight = kKernel[static_cast<std::size_t>(std::abs(dx))] *
                                          kKernel[static_cast<std::size_t>(std::abs(dy))];
                            if (qi != ci) {
                                const Real dl = luminance(current[qi]) - lum_c;
                                weight *= std::exp(-(dl * dl) / (sigma_c * sigma_c * (Real(1) + lum_c * lum_c)));
                                weight *= std::pow(std::max(Real(0), dot(nc, fb.normal[qi])), s.sigma_normal);
                                const Real dq = fb.depth[qi];
                                if (std::isinf(dc) != std::isinf(dq)) {
                                    weight = 0;  // sky never mixes with geometry
                                } else if (!std::isinf(dc)) {
                                    const Real rel = std::fabs(dc - dq) / std::max(dc, Real(1e-3));
                                    weight *= std::exp(-(rel * rel) / (depth_tol * depth_tol));
                                }
                            }
                            sum += current[qi] * weight;
                            weight_sum += weight;
                        }
                    }
                    next[ci] = weight_sum > 0 ? sum / weight_sum : current[ci];
                }
            }
        });
        current.swap(next);
    }

    for (std::size_t i = 0; i < n; ++i) {
        current[i] = current[i] * vmax(fb.albedo[i], Vec3(Real(0.02)));
    }
    return current;
}

}  // namespace lumen
