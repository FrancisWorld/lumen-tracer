#include <atomic>
#include <mutex>
#include <chrono>
#include <set>
#include <stdexcept>
#include <thread>

#include "lumen/denoise.hpp"
#include "lumen/renderer.hpp"
#include "lumen/rng.hpp"
#include "lumen/scheduler.hpp"
#include "lumen/tonemap.hpp"
#include "testing.hpp"

using namespace lumen;

namespace {

/// A small scene with glass, metal, a light and a checkered floor: uneven work per tile.
struct Demo {
    Scene scene;
    Camera camera;
    Demo() {
        const auto checker = scene.add_texture(CheckerTexture{Vec3(0.8F), Vec3(0.1F), 4});
        const auto white = scene.add_texture(ConstantTexture{Vec3(0.9F)});
        Material floor;
        floor.base_texture = checker;
        floor.specular = 0;
        const auto floor_id = scene.add_material(floor);
        Material glass;
        glass.kind = Material::Kind::Dielectric;
        glass.base_texture = white;
        const auto glass_id = scene.add_material(glass);
        Material metal;
        metal.base_texture = white;
        metal.metallic = 1;
        metal.roughness = 0.3F;
        const auto metal_id = scene.add_material(metal);
        Material lamp;
        lamp.base_texture = white;
        lamp.emission = Vec3(15);
        const auto lamp_id = scene.add_material(lamp);
        scene.add_sphere({{0, -1000, 0}, 1000, floor_id});
        scene.add_sphere({{-1.2F, 0.7F, 0}, 0.7F, glass_id});
        scene.add_sphere({{1.2F, 0.7F, 0}, 0.7F, metal_id});
        scene.add_sphere({{0, 4, 1}, 0.8F, lamp_id});
        Environment sky;
        sky.kind = Environment::Kind::Gradient;
        sky.horizon = Vec3(0.6F);
        sky.zenith = Vec3(0.2F, 0.3F, 0.6F);
        scene.set_environment(sky);
        scene.build();
        CameraParams p;
        p.look_from = {0, 2, 6};
        p.look_at = {0, 0.7F, 0};
        p.aperture = 0.1F;
        camera = Camera(p, 1.5F);
    }
};

RenderSettings small_settings() {
    RenderSettings s;
    s.width = 48;
    s.height = 32;
    s.samples_per_pixel = 6;
    s.seed = 77;
    s.path.max_depth = 6;
    return s;
}

}  // namespace

TEST_CASE(scheduler, runs_every_task_exactly_once) {
    for (const unsigned threads : {1U, 2U, 4U, 9U}) {
        const std::size_t n = 257;
        std::vector<std::atomic<int>> counts(n);
        const SchedulerStats stats = run_work_stealing(n, threads, [&](std::size_t t, unsigned) { ++counts[t]; });
        for (std::size_t i = 0; i < n; ++i) {
            CHECK_EQ(counts[i].load(), 1);
        }
        std::size_t total = 0;
        for (const auto c : stats.tasks_per_worker) {
            total += c;
        }
        CHECK_EQ(total, n);
        CHECK_EQ(stats.workers, threads);
    }
}

TEST_CASE(scheduler, idle_workers_steal_from_a_slow_neighbour) {
    // Tasks 0..15 are slow and all start in worker 0's queue; the other workers must steal them.
    std::set<unsigned> workers_on_slow_tasks;
    std::mutex m;
    const SchedulerStats stats = run_work_stealing(64, 4, [&](std::size_t t, unsigned w) {
        if (t < 16) {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            const std::lock_guard<std::mutex> lock(m);
            workers_on_slow_tasks.insert(w);
        }
    });
    CHECK(stats.steals > 0);
    CHECK(workers_on_slow_tasks.size() > 1);
    for (const auto c : stats.tasks_per_worker) {
        CHECK(c > 0);
    }
}

TEST_CASE(scheduler, propagates_the_first_exception_and_stops) {
    std::atomic<int> started{0};
    bool threw = false;
    try {
        run_work_stealing(10000, 4, [&](std::size_t t, unsigned) {
            ++started;
            if (t == 3) {
                throw std::runtime_error("boom");
            }
        });
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()) == "boom";
    }
    CHECK(threw);
    CHECK(started.load() < 10000);
}

TEST_CASE(scheduler, handles_empty_and_oversubscribed_workloads) {
    CHECK_EQ(run_work_stealing(0, 8, [](std::size_t, unsigned) {}).tasks, 0U);
    const SchedulerStats few = run_work_stealing(2, 16, [](std::size_t, unsigned) {});
    CHECK_EQ(few.workers, 2U);  // never more workers than tasks
    CHECK(run_work_stealing(5, 0, [](std::size_t, unsigned) {}).workers >= 1);
}

TEST_CASE(render, output_is_bit_identical_for_any_thread_count_and_tile_size) {
    const Demo demo;
    RenderSettings s = small_settings();
    s.threads = 1;
    s.tile_size = 16;
    const std::uint64_t reference = digest(render(demo.scene, demo.camera, s));
    for (const unsigned threads : {2U, 3U, 7U}) {
        for (const std::uint32_t tile : {5U, 16U, 64U}) {
            s.threads = threads;
            s.tile_size = tile;
            CHECK_EQ(digest(render(demo.scene, demo.camera, s)), reference);
        }
    }
    s.seed = 78;
    CHECK(digest(render(demo.scene, demo.camera, s)) != reference);  // the seed really matters
}

TEST_CASE(render, produces_finite_plausible_pixels_and_reports_progress) {
    const Demo demo;
    RenderSettings s = small_settings();
    s.threads = 2;
    s.tile_size = 8;
    std::vector<double> seen;
    RenderStats stats;
    const Framebuffer fb = render(demo.scene, demo.camera, s, [&](double f) { seen.push_back(f); }, &stats);
    REQUIRE(fb.color.size() == 48U * 32U);
    double mean = 0;
    for (const Vec3& c : fb.color) {
        CHECK(is_finite(c));
        CHECK(c.x >= 0 && c.y >= 0 && c.z >= 0);
        mean += luminance(c);
    }
    CHECK(mean / static_cast<double>(fb.color.size()) > 0.05);
    CHECK_EQ(seen.size(), 24U);  // 6 x 4 tiles
    CHECK_NEAR(seen.back(), 1.0, 1e-12);
    for (std::size_t i = 1; i < seen.size(); ++i) {
        CHECK(seen[i] >= seen[i - 1]);
    }
    CHECK_EQ(stats.paths, 48ULL * 32ULL * 6ULL);
    CHECK(stats.seconds > 0);
    CHECK_EQ(stats.scheduler.tasks, 24U);
    // Bottom rows see the floor: guides must be populated there and empty for sky.
    CHECK(std::isfinite(fb.depth[31 * 48 + 24]));
    CHECK(std::isinf(fb.depth[0]));
}

TEST_CASE(render, empty_scene_shows_the_sky_exactly) {
    Scene scene;
    Environment e;
    e.horizon = {0.1F, 0.2F, 0.3F};
    scene.set_environment(e);
    scene.build();
    RenderSettings s = small_settings();
    s.threads = 2;
    const Framebuffer fb = render(scene, Camera(), s);
    for (const Vec3& c : fb.color) {
        CHECK_NEAR(c.y, 0.2, 1e-6);
    }
}

TEST_CASE(render, rejects_invalid_settings) {
    const Demo demo;
    RenderSettings s = small_settings();
    s.width = 0;
    CHECK_THROWS(render(demo.scene, demo.camera, s));
    s = small_settings();
    s.samples_per_pixel = 0;
    CHECK_THROWS(render(demo.scene, demo.camera, s));
    s = small_settings();
    s.tile_size = 0;
    CHECK_THROWS(render(demo.scene, demo.camera, s));
    s = small_settings();
    s.path.max_depth = -1;
    CHECK_THROWS(render(demo.scene, demo.camera, s));
}

namespace {

Framebuffer flat_framebuffer(std::uint32_t w, std::uint32_t h) {
    Framebuffer fb;
    fb.width = w;
    fb.height = h;
    const std::size_t n = static_cast<std::size_t>(w) * h;
    fb.color.assign(n, Vec3(0.5F));
    fb.albedo.assign(n, Vec3(1));
    fb.normal.assign(n, Vec3(0, 0, 1));
    fb.depth.assign(n, 5.0F);
    return fb;
}

double mse(const std::vector<Vec3>& a, Real truth_left, Real truth_right, std::uint32_t w, std::uint32_t split) {
    double sum = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Real truth = (i % w) < split ? truth_left : truth_right;
        const double d = static_cast<double>(a[i].x - truth);
        sum += d * d;
    }
    return sum / static_cast<double>(a.size());
}

}  // namespace

TEST_CASE(denoise, leaves_clean_images_untouched) {
    Framebuffer fb = flat_framebuffer(32, 24);
    const auto out = denoise(fb, {});
    for (const Vec3& c : out) {
        CHECK_NEAR(c.x, 0.5, 1e-5);
    }
    DenoiseSettings off;
    off.iterations = 0;
    CHECK_NEAR(denoise(fb, off)[5].x, 0.5, 1e-6);
}

TEST_CASE(denoise, removes_noise_but_keeps_geometric_edges) {
    // Two flat surfaces with different normals and depth; radiance 0.2 on the left, 0.8 on the right, plus noise.
    const std::uint32_t w = 64;
    const std::uint32_t h = 48;
    const std::uint32_t split = 32;
    Framebuffer fb = flat_framebuffer(w, h);
    Pcg32 rng(5);
    for (std::uint32_t y = 0; y < h; ++y) {
        for (std::uint32_t x = 0; x < w; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * w + x;
            const bool left = x < split;
            const Real truth = left ? 0.2F : 0.8F;
            fb.color[i] = Vec3(truth + (rng.next_real() - 0.5F) * 0.3F);
            fb.normal[i] = left ? Vec3(0, 0, 1) : normalize(Vec3(1, 0, 1));
            fb.depth[i] = left ? 4.0F : 6.0F;
        }
    }
    const double before = mse(fb.color, 0.2F, 0.8F, w, split);
    const auto out = denoise(fb, {});
    const double after = mse(out, 0.2F, 0.8F, w, split);
    CHECK(after < before * 0.2);  // large noise reduction

    // No bleeding across the edge: pixels beside the boundary keep their own side's value.
    double left_edge = 0;
    double right_edge = 0;
    for (std::uint32_t y = 8; y < h - 8; ++y) {
        left_edge += out[static_cast<std::size_t>(y) * w + split - 1].x;
        right_edge += out[static_cast<std::size_t>(y) * w + split].x;
    }
    left_edge /= (h - 16);
    right_edge /= (h - 16);
    CHECK_NEAR(left_edge, 0.2, 0.02);
    CHECK_NEAR(right_edge, 0.8, 0.02);

    // Naive blur without guides would smear the edge: confirm guides are what protect it.
    Framebuffer unguided = fb;
    std::fill(unguided.normal.begin(), unguided.normal.end(), Vec3(0, 0, 1));
    std::fill(unguided.depth.begin(), unguided.depth.end(), 5.0F);
    DenoiseSettings loose;
    loose.sigma_color = 50;  // colour edge-stopping effectively off
    const auto smeared = denoise(unguided, loose);
    CHECK(std::fabs(smeared[static_cast<std::size_t>(20) * w + split - 1].x - 0.2F) > 0.05F);
}

TEST_CASE(denoise, is_deterministic_across_thread_counts_and_preserves_albedo_texture) {
    Framebuffer fb = flat_framebuffer(40, 40);
    Pcg32 rng(8);
    for (std::size_t i = 0; i < fb.color.size(); ++i) {
        const bool stripe = ((i % 40) / 4) % 2 == 0;
        fb.albedo[i] = Vec3(stripe ? 0.9F : 0.1F);
        // irradiance 1 plus noise, modulated by albedo
        fb.color[i] = fb.albedo[i] * (1.0F + (rng.next_real() - 0.5F) * 0.2F);
    }
    DenoiseSettings one;
    one.threads = 1;
    DenoiseSettings many;
    many.threads = 4;
    const auto a = denoise(fb, one);
    const auto b = denoise(fb, many);
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK_EQ(a[i].x, b[i].x);
    }
    // Stripes survive because filtering happens on demodulated irradiance.
    const Real bright = a[20 * 40 + 1].x;
    const Real dark = a[20 * 40 + 5].x;
    CHECK(bright > 0.8F);
    CHECK(dark < 0.2F);
}

TEST_CASE(denoise, validates_input_and_keeps_sky_separate) {
    Framebuffer fb = flat_framebuffer(8, 8);
    fb.albedo.pop_back();
    CHECK_THROWS(denoise(fb, {}));
    DenoiseSettings bad;
    bad.iterations = 99;
    CHECK_THROWS(denoise(flat_framebuffer(8, 8), bad));

    Framebuffer sky = flat_framebuffer(8, 8);
    for (std::size_t i = 0; i < sky.color.size(); ++i) {
        if (i % 8 >= 4) {
            sky.depth[i] = kInf;
            sky.color[i] = Vec3(5);
        }
    }
    const auto out = denoise(sky, {});
    CHECK_NEAR(out[3].x, 0.5, 1e-4);
    CHECK_NEAR(out[4].x, 5.0, 1e-3);
}

TEST_CASE(tonemap, aces_curve_properties) {
    CHECK_NEAR(aces_tonemap(Vec3(0)).x, 0.0, 1e-6);
    CHECK_NEAR(aces_tonemap(Vec3(1)).x, 0.8038, 1e-3);
    CHECK_NEAR(aces_tonemap(Vec3(1000)).x, 1.0, 1e-6);  // saturates, never exceeds 1
    CHECK_NEAR(aces_tonemap(Vec3(-3)).x, 0.0, 1e-6);
    Real previous = -1;
    for (int i = 0; i <= 100; ++i) {
        const Real v = aces_tonemap(Vec3(static_cast<Real>(i) * 0.1F)).x;
        CHECK(v >= previous);
        previous = v;
    }
}

TEST_CASE(tonemap, converts_to_8_bit_srgb_with_exposure) {
    const std::vector<Vec3> hdr = {Vec3(0), Vec3(0.18F), Vec3(100)};
    const Image8 img = to_srgb8(hdr, 3, 1, 1.0F);
    CHECK_EQ(img.pixels[0], 0);
    CHECK_EQ(img.pixels[6], 255);
    CHECK(img.pixels[3] > 90 && img.pixels[3] < 150);
    const Image8 dark = to_srgb8(hdr, 3, 1, 0.25F);
    CHECK(dark.pixels[3] < img.pixels[3]);
    CHECK_THROWS(to_srgb8(hdr, 2, 2, 1.0F));
}
