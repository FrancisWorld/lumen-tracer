#include "lumen/renderer.hpp"

#include <bit>
#include <chrono>
#include <mutex>

#include "lumen/error.hpp"

namespace lumen {

void RenderSettings::validate() const {
    if (width == 0 || height == 0 || width > 16384 || height > 16384) {
        throw InputError("render: width and height must be between 1 and 16384");
    }
    if (samples_per_pixel == 0 || samples_per_pixel > 1'000'000) {
        throw InputError("render: samples per pixel must be between 1 and 1000000");
    }
    if (tile_size == 0 || tile_size > 1024) {
        throw InputError("render: tile size must be between 1 and 1024");
    }
    if (path.max_depth < 0 || path.max_depth > 1000 || path.roulette_depth < 0 || path.sample_clamp < 0) {
        throw InputError("render: invalid path settings");
    }
}

Framebuffer render(const Scene& scene, const Camera& camera, const RenderSettings& settings,
                   const std::function<void(double)>& progress, RenderStats* stats) {
    settings.validate();
    const auto start = std::chrono::steady_clock::now();

    Framebuffer fb;
    fb.width = settings.width;
    fb.height = settings.height;
    const std::size_t pixels = static_cast<std::size_t>(settings.width) * settings.height;
    fb.color.assign(pixels, Vec3{});
    fb.albedo.assign(pixels, Vec3{});
    fb.normal.assign(pixels, Vec3{});
    fb.depth.assign(pixels, kInf);

    const std::uint32_t tiles_x = (settings.width + settings.tile_size - 1) / settings.tile_size;
    const std::uint32_t tiles_y = (settings.height + settings.tile_size - 1) / settings.tile_size;
    const std::size_t tile_count = static_cast<std::size_t>(tiles_x) * tiles_y;
    const Vec2 image_size{static_cast<Real>(settings.width), static_cast<Real>(settings.height)};

    std::mutex progress_mutex;
    std::size_t tiles_done = 0;

    auto render_tile = [&](std::size_t tile, unsigned /*worker*/) {
        const std::uint32_t x0 = static_cast<std::uint32_t>(tile % tiles_x) * settings.tile_size;
        const std::uint32_t y0 = static_cast<std::uint32_t>(tile / tiles_x) * settings.tile_size;
        const std::uint32_t x1 = std::min(x0 + settings.tile_size, settings.width);
        const std::uint32_t y1 = std::min(y0 + settings.tile_size, settings.height);
        for (std::uint32_t y = y0; y < y1; ++y) {
            for (std::uint32_t x = x0; x < x1; ++x) {
                Pcg32 rng = pixel_rng(settings.seed, x, y);
                Vec3 color;
                Vec3 albedo;
                Vec3 normal;
                double depth_sum = 0;
                std::uint32_t depth_hits = 0;
                for (std::uint32_t s = 0; s < settings.samples_per_pixel; ++s) {
                    const Vec2 jitter = rng.next_vec2();
                    const Vec2 lens = rng.next_vec2();
                    const Ray ray = camera.generate_ray({static_cast<Real>(x) + jitter.x, static_cast<Real>(y) + jitter.y},
                                                        image_size, lens);
                    const PathSample sample = trace_path(scene, ray, rng, settings.path);
                    color += sample.radiance;
                    albedo += sample.guides.albedo;
                    normal += sample.guides.normal;
                    if (std::isfinite(sample.guides.depth)) {
                        depth_sum += static_cast<double>(sample.guides.depth);
                        ++depth_hits;
                    }
                }
                const Real inv = Real(1) / static_cast<Real>(settings.samples_per_pixel);
                const std::size_t i = static_cast<std::size_t>(y) * settings.width + x;
                fb.color[i] = color * inv;
                fb.albedo[i] = albedo * inv;
                const Real nl = length(normal);
                fb.normal[i] = nl > 0 ? normal / nl : Vec3{};
                fb.depth[i] = depth_hits > 0 ? static_cast<Real>(depth_sum / depth_hits) : kInf;
            }
        }
        if (progress) {
            const std::lock_guard<std::mutex> lock(progress_mutex);
            ++tiles_done;
            progress(static_cast<double>(tiles_done) / static_cast<double>(tile_count));
        }
    };

    SchedulerStats sched = run_work_stealing(tile_count, settings.threads, render_tile);

    if (stats != nullptr) {
        stats->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        stats->paths = static_cast<std::uint64_t>(pixels) * settings.samples_per_pixel;
        stats->scheduler = std::move(sched);
    }
    return fb;
}

std::uint64_t digest(const Framebuffer& fb) {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    auto mix = [&h](Real v) {
        h ^= std::bit_cast<std::uint32_t>(v);
        h *= 0x100000001b3ULL;
    };
    for (const Vec3& v : fb.color) {
        mix(v.x);
        mix(v.y);
        mix(v.z);
    }
    for (const Vec3& v : fb.albedo) {
        mix(v.x);
        mix(v.y);
        mix(v.z);
    }
    for (const Vec3& v : fb.normal) {
        mix(v.x);
        mix(v.y);
        mix(v.z);
    }
    for (const Real d : fb.depth) {
        mix(d);
    }
    return h;
}

}  // namespace lumen
