// lumen: command line front end.
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include "lumen/denoise.hpp"
#include "lumen/error.hpp"
#include "lumen/scene_loader.hpp"
#include "lumen/tonemap.hpp"

namespace {

constexpr const char* kVersion = "0.1.0";

void print_usage(std::FILE* out) {
    std::fprintf(out, "lumen %s - a physically based path tracer\n", kVersion);
    std::fputs(
        "\n"
        "usage:\n"
        "  lumen render <scene.json> [options]   render a scene to a PNG\n"
        "  lumen info <scene.json>               validate a scene and print statistics\n"
        "  lumen --version | --help\n"
        "\n"
        "render options (override the scene's settings):\n"
        "  -o, --output <file>   output PNG path (default: out.png)\n"
        "  --spp <n>             samples per pixel\n"
        "  --width <n>  --height <n>\n"
        "  --depth <n>           maximum bounces\n"
        "  --threads <n>         worker threads (default: all cores)\n"
        "  --seed <n>            random seed; same seed gives bit-identical output\n"
        "  --tile <n>            tile edge in pixels (default 32)\n"
        "  --exposure <x>        linear exposure multiplier\n"
        "  --denoise | --no-denoise\n"
        "  --quiet               no progress output\n",
        out);
}

struct UsageError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

long parse_count(std::string_view name, const char* text, long lo, long hi) {
    long v = 0;
    const auto* end = text + std::strlen(text);
    const auto [ptr, ec] = std::from_chars(text, end, v);
    if (ec != std::errc() || ptr != end || v < lo || v > hi) {
        throw UsageError(std::string(name) + " expects a whole number between " + std::to_string(lo) + " and " + std::to_string(hi));
    }
    return v;
}

double parse_real(std::string_view name, const char* text) {
    char* end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text || *end != '\0' || !(v > 0)) {
        throw UsageError(std::string(name) + " expects a positive number");
    }
    return v;
}

struct RenderOptions {
    std::string scene;
    std::string output = "out.png";
    bool quiet = false;
    long spp = 0;
    long width = 0;
    long height = 0;
    long depth = -1;
    long threads = -1;
    long seed = -1;
    long tile = 0;
    double exposure = 0;
    int denoise = -1;  // -1 = scene default
};

RenderOptions parse_render_args(int argc, char** argv) {
    RenderOptions o;
    for (int i = 2; i < argc; ++i) {
        const std::string_view a = argv[i];
        auto value = [&]() -> const char* {
            if (i + 1 >= argc) {
                throw UsageError(std::string(a) + " needs a value");
            }
            return argv[++i];
        };
        if (a == "-o" || a == "--output") {
            o.output = value();
        } else if (a == "--spp") {
            o.spp = parse_count(a, value(), 1, 1'000'000);
        } else if (a == "--width") {
            o.width = parse_count(a, value(), 1, 16384);
        } else if (a == "--height") {
            o.height = parse_count(a, value(), 1, 16384);
        } else if (a == "--depth") {
            o.depth = parse_count(a, value(), 0, 1000);
        } else if (a == "--threads") {
            o.threads = parse_count(a, value(), 0, 1024);
        } else if (a == "--seed") {
            o.seed = parse_count(a, value(), 0, 1L << 52);
        } else if (a == "--tile") {
            o.tile = parse_count(a, value(), 1, 1024);
        } else if (a == "--exposure") {
            o.exposure = parse_real(a, value());
        } else if (a == "--denoise") {
            o.denoise = 1;
        } else if (a == "--no-denoise") {
            o.denoise = 0;
        } else if (a == "--quiet") {
            o.quiet = true;
        } else if (!a.empty() && a[0] == '-') {
            throw UsageError("unknown option " + std::string(a));
        } else if (o.scene.empty()) {
            o.scene = std::string(a);
        } else {
            throw UsageError("unexpected argument " + std::string(a));
        }
    }
    if (o.scene.empty()) {
        throw UsageError("missing scene file");
    }
    return o;
}

int run_render(const RenderOptions& o) {
    using clock = std::chrono::steady_clock;
    lumen::SceneDescription d = lumen::load_scene_file(o.scene);
    if (o.width > 0) d.render.width = static_cast<std::uint32_t>(o.width);
    if (o.height > 0) d.render.height = static_cast<std::uint32_t>(o.height);
    if (o.width > 0 || o.height > 0) {
        d.camera = lumen::Camera(d.camera_params, static_cast<lumen::Real>(d.render.width) / static_cast<lumen::Real>(d.render.height));
    }
    if (o.spp > 0) d.render.samples_per_pixel = static_cast<std::uint32_t>(o.spp);
    if (o.depth >= 0) d.render.path.max_depth = static_cast<int>(o.depth);
    if (o.threads >= 0) d.render.threads = static_cast<unsigned>(o.threads);
    if (o.seed >= 0) d.render.seed = static_cast<std::uint64_t>(o.seed);
    if (o.tile > 0) d.render.tile_size = static_cast<std::uint32_t>(o.tile);
    if (o.exposure > 0) d.exposure = static_cast<lumen::Real>(o.exposure);
    if (o.denoise >= 0) d.denoise = o.denoise == 1;

    const auto& r = d.render;
    if (!o.quiet) {
        std::fprintf(stderr, "scene %s: %zu primitives, %zu lights, BVH %zu nodes\n", o.scene.c_str(),
                     d.scene.primitive_count(), d.scene.light_count(), d.scene.bvh().stats().node_count);
        std::fprintf(stderr, "rendering %ux%u at %u spp, max depth %d\n", r.width, r.height, r.samples_per_pixel,
                     r.path.max_depth);
    }

    std::mutex print_mutex;
    int last_percent = -1;
    auto progress = [&](double f) {
        if (o.quiet) {
            return;
        }
        const int percent = static_cast<int>(f * 100);
        const std::lock_guard<std::mutex> lock(print_mutex);
        if (percent != last_percent && percent % 10 == 0) {
            last_percent = percent;
            std::fprintf(stderr, "  %3d%%\n", percent);
        }
    };

    lumen::RenderStats stats;
    lumen::Framebuffer fb = lumen::render(d.scene, d.camera, r, progress, &stats);

    std::vector<lumen::Vec3> image = fb.color;
    double denoise_seconds = 0;
    if (d.denoise) {
        const auto t0 = clock::now();
        lumen::DenoiseSettings ds;
        ds.threads = r.threads;
        image = lumen::denoise(fb, ds);
        denoise_seconds = std::chrono::duration<double>(clock::now() - t0).count();
    }
    lumen::write_png_file(o.output, lumen::to_srgb8(image, fb.width, fb.height, d.exposure));

    const double mpaths = static_cast<double>(stats.paths) / stats.seconds / 1e6;
    std::printf("rendered %ux%u @ %u spp in %.2f s (%.2f Mpaths/s, %u threads, %zu tile steals)%s\n", r.width, r.height,
                r.samples_per_pixel, stats.seconds, mpaths, stats.scheduler.workers, stats.scheduler.steals,
                d.denoise ? "" : ", no denoise");
    if (d.denoise) {
        std::printf("denoised in %.2f s\n", denoise_seconds);
    }
    std::printf("wrote %s\n", o.output.c_str());
    return 0;
}

int run_info(const char* path) {
    const lumen::SceneDescription d = lumen::load_scene_file(path);
    const lumen::BvhStats b = d.scene.bvh().stats();
    std::printf("primitives: %zu\nlights: %zu\nbvh nodes: %zu (leaves %zu, depth %zu, max leaf %zu)\n"
                "image: %ux%u, %u spp, max depth %d, denoise %s\n",
                d.scene.primitive_count(), d.scene.light_count(), b.node_count, b.leaf_count, b.max_depth,
                b.max_leaf_size, d.render.width, d.render.height, d.render.samples_per_pixel, d.render.path.max_depth,
                d.denoise ? "on" : "off");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            print_usage(stderr);
            return 2;
        }
        const std::string_view cmd = argv[1];
        if (cmd == "--help" || cmd == "-h") {
            print_usage(stdout);
            return 0;
        }
        if (cmd == "--version") {
            std::printf("lumen %s\n", kVersion);
            return 0;
        }
        if (cmd == "render") {
            return run_render(parse_render_args(argc, argv));
        }
        if (cmd == "info") {
            if (argc != 3) {
                throw UsageError("info takes exactly one scene file");
            }
            return run_info(argv[2]);
        }
        throw UsageError("unknown command " + std::string(cmd));
    } catch (const UsageError& e) {
        std::fprintf(stderr, "lumen: %s\n\n", e.what());
        print_usage(stderr);
        return 2;
    } catch (const lumen::InputError& e) {
        std::fprintf(stderr, "lumen: %s\n", e.what());
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "lumen: error: %s\n", e.what());
        return 1;
    }
}
