// Tiled, multithreaded renderer. Every pixel owns a random stream seeded from
// (seed, x, y), so the output does not depend on thread count or tile order.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "lumen/camera.hpp"
#include "lumen/integrator.hpp"
#include "lumen/scheduler.hpp"

namespace lumen {

struct RenderSettings {
    std::uint32_t width = 640;
    std::uint32_t height = 360;
    std::uint32_t samples_per_pixel = 64;
    std::uint32_t tile_size = 32;
    unsigned threads = 0;  // 0 = hardware concurrency
    std::uint64_t seed = 1;
    PathSettings path;

    /// Throws InputError when a value is out of range.
    void validate() const;
};

struct Framebuffer {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<Vec3> color;   // linear radiance, averaged over samples
    std::vector<Vec3> albedo;  // denoiser guides
    std::vector<Vec3> normal;
    std::vector<Real> depth;
};

struct RenderStats {
    double seconds = 0;
    std::uint64_t paths = 0;
    SchedulerStats scheduler;
};

/// `progress` receives the fraction of tiles finished; it may be called from any worker (calls are serialised).
Framebuffer render(const Scene& scene, const Camera& camera, const RenderSettings& settings,
                   const std::function<void(double)>& progress = {}, RenderStats* stats = nullptr);

/// Order-sensitive 64-bit digest of every buffer, used to prove determinism.
std::uint64_t digest(const Framebuffer& fb);

}  // namespace lumen
