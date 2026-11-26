#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "lumen/bvh.hpp"
#include "lumen/denoise.hpp"
#include "lumen/fileio.hpp"
#include "lumen/scene_loader.hpp"
#include "lumen/tonemap.hpp"
#include "testing.hpp"

using namespace lumen;

namespace {

const std::filesystem::path kRoot = LUMEN_SOURCE_DIR;

SceneDescription from_text(const char* text) { return build_scene(parse_json(text), kRoot / "scenes"); }

std::string error_of(const char* text) {
    try {
        (void)from_text(text);
    } catch (const InputError& e) {
        return e.what();
    }
    return "";
}

bool mentions(const std::string& message, const char* fragment) { return message.find(fragment) != std::string::npos; }

constexpr const char* kMinimal = R"({
  "materials": { "m": { "base_color": [0.5, 0.5, 0.5] } },
  "objects": [ { "type": "sphere", "center": [0, 0, 0], "radius": 1, "material": "m" } ]
})";

}  // namespace

TEST_CASE(loader, minimal_scene_gets_sensible_defaults) {
    const SceneDescription d = from_text(kMinimal);
    CHECK_EQ(d.scene.primitive_count(), 1U);
    CHECK_EQ(d.render.width, 640U);
    CHECK_EQ(d.render.height, 360U);
    CHECK(!d.denoise);
    CHECK_NEAR(d.exposure, 1.0, 0);
}

TEST_CASE(loader, reads_every_section) {
    const SceneDescription d = from_text(R"({
      "settings": { "width": 100, "height": 50, "samples": 7, "max_depth": 3, "seed": 9, "exposure": 2, "denoise": true,
                    "tile_size": 16, "roulette_depth": 1, "sample_clamp": 5 },
      "camera": { "from": [0, 1, 5], "at": [0, 0, 0], "fov": 60, "aperture": 0.1, "focus_distance": 4 },
      "environment": { "type": "gradient", "horizon": [1, 1, 1], "zenith": [0, 0, 1], "intensity": 0.5 },
      "textures": {
        "c": { "type": "checker", "a": [1, 0, 0], "b": [0, 1, 0], "scale": 4 },
        "k": { "type": "constant", "color": [0.1, 0.2, 0.3] }
      },
      "materials": {
        "lit": { "base_color": "c", "metallic": 0.3, "roughness": 0.2, "specular": 0.7 },
        "lamp": { "base_color": [1, 1, 1], "emission": [1, 0.5, 0.25], "emission_strength": 4 },
        "glass": { "type": "dielectric", "ior": 1.33, "tint": [0.9, 1, 1] },
        "tex": { "base_color": "k" }
      },
      "objects": [
        { "type": "sphere", "center": [0, 0, 0], "radius": 1, "material": "lit" },
        { "type": "triangle", "vertices": [[0, 0, 0], [1, 0, 0], [0, 1, 0]], "material": "glass" },
        { "type": "quad", "corner": [0, 3, 0], "u": [1, 0, 0], "v": [0, 0, 1], "material": "lamp" },
        { "type": "mesh", "path": "../assets/models/rock.obj", "material": "tex", "translate": [3, 0, 0], "scale": [1, 2, 1], "rotate": [0, 45, 0], "smooth": false }
      ]
    })");
    CHECK_EQ(d.render.width, 100U);
    CHECK_EQ(d.render.samples_per_pixel, 7U);
    CHECK_EQ(d.render.path.max_depth, 3);
    CHECK_EQ(d.render.seed, 9ULL);
    CHECK_EQ(d.render.tile_size, 16U);
    CHECK_NEAR(d.render.path.sample_clamp, 5.0, 0);
    CHECK(d.denoise);
    CHECK_NEAR(d.exposure, 2.0, 0);
    CHECK_NEAR(d.camera_params.vertical_fov_degrees, 60.0, 0);
    CHECK_NEAR(d.camera.focus_distance(), 4.0, 0);
    CHECK_EQ(d.scene.light_count(), 2U);  // the quad is two emissive triangles
    CHECK(d.scene.primitive_count() > 1000);  // sphere + triangle + quad + rock mesh
    CHECK_NEAR(d.scene.environment({0, 1, 0}).z, 0.5, 1e-6);
}

TEST_CASE(loader, emission_strength_scales_colour) {
    SceneDescription d = from_text(R"({
      "materials": { "lamp": { "emission": [1, 0.5, 0.25], "emission_strength": 4 } },
      "objects": [ { "type": "sphere", "center": [0, 0, -3], "radius": 1, "material": "lamp" } ] })");
    Hit hit;
    REQUIRE(d.scene.intersect({{0, 0, 0}, {0, 0, -1}}, kInf, hit));
    CHECK_NEAR(d.scene.material(hit.material).emission.x, 4.0, 1e-6);
    CHECK_NEAR(d.scene.material(hit.material).emission.z, 1.0, 1e-6);
}

TEST_CASE(loader, errors_name_the_offending_path) {
    CHECK(mentions(error_of(R"({"bogus": 1})"), "unknown key \"bogus\""));
    CHECK(mentions(error_of(R"({"objects": [{"type": "sphere", "center": [0,0,0], "radius": 1, "material": "nope"}]})"),
                   "objects[0].material: unknown material \"nope\""));
    CHECK(mentions(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "sphere", "center": [0,0], "radius": 1, "material": "m"}]})"),
                   "objects[0].center: expected an array of three numbers"));
    CHECK(mentions(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "sphere", "center": [0,0,0], "radius": -1, "material": "m"}]})"),
                   "radius: must be greater than zero"));
    CHECK(mentions(error_of(R"({"materials": {"m": {"roughness": 3}}})"), "materials.m.roughness: must be between 0 and 1"));
    CHECK(mentions(error_of(R"({"materials": {"m": {"base_color": [1, -1, 0]}}})"), "must not be negative"));
    CHECK(mentions(error_of(R"({"materials": {"m": {"base_color": "ghost"}}})"), "unknown texture \"ghost\""));
    CHECK(mentions(error_of(R"({"materials": {"m": {"type": "plasma"}}})"), "unknown material \"plasma\""));
    CHECK(mentions(error_of(R"({"materials": {"g": {"type": "dielectric", "tint": [2, 1, 1]}}})"), "cannot amplify"));
    CHECK(mentions(error_of(R"({"materials": {"g": {"type": "dielectric", "ior": 9}}})"), "ior"));
    CHECK(mentions(error_of(R"({"objects": [{"type": "cube"}]})"), "unknown object \"cube\""));
    CHECK(mentions(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "triangle", "vertices": [[0,0,0],[1,0,0],[2,0,0]], "material": "m"}]})"),
                   "zero area"));
    CHECK(mentions(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "quad", "corner": [0,0,0], "u": [1,0,0], "v": [2,0,0], "material": "m"}]})"),
                   "non-degenerate"));
    CHECK(mentions(error_of(R"({"settings": {"width": 0}})"), "settings.width"));
    CHECK(mentions(error_of(R"({"settings": {"samples": 1.5}})"), "whole number"));
    CHECK(mentions(error_of(R"({"settings": {"samples": -4}})"), "settings.samples"));
    CHECK(mentions(error_of(R"({"settings": {"denoise": "yes"}})"), "expected boolean"));
    CHECK(mentions(error_of(R"({"settings": {"typo": 1}})"), "unknown key \"typo\""));
    CHECK(mentions(error_of(R"({"camera": {"from": [0,0,0], "at": [0,0,0]}})"), "coincide"));
    CHECK(mentions(error_of(R"({"environment": {"type": "hdri"}})"), "unknown environment"));
    CHECK(mentions(error_of(R"({"objects": 5})"), "objects: expected an array"));
    CHECK(mentions(error_of("[]"), "expected an object"));
    CHECK(mentions(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "mesh", "path": "missing.obj", "material": "m"}]})"),
                   "cannot open"));
    CHECK(mentions(error_of(R"({"textures": {"t": {"type": "image", "path": "missing.png"}}})"), "missing.png"));
    CHECK(mentions(error_of(R"({"textures": {"t": {"type": "image", "path": "../assets/textures/mosaic.png", "repeat": [0, 1]}}})"),
                   "repeat"));
}

TEST_CASE(loader, file_errors_carry_the_file_name_and_json_position) {
    try {
        (void)load_scene_file((kRoot / "CMakeLists.txt").string());  // not JSON
        CHECK(false);
    } catch (const InputError& e) {
        CHECK(mentions(e.what(), "CMakeLists.txt"));
        CHECK(mentions(e.what(), "line 1"));
    }
    CHECK_THROWS(load_scene_file("/nonexistent/scene.json"));
}

TEST_CASE(loader, every_shipped_scene_loads_and_is_lit) {
    std::size_t found = 0;
    for (const auto& entry : std::filesystem::directory_iterator(kRoot / "scenes")) {
        if (entry.path().extension() != ".json") {
            continue;
        }
        ++found;
        const SceneDescription d = load_scene_file(entry.path().string());
        CHECK(d.scene.primitive_count() > 5);
        CHECK(d.scene.light_count() > 0);
        CHECK_EQ(d.render.width, 640U);
        CHECK_EQ(d.render.height, 360U);
        CHECK(d.denoise);
    }
    CHECK(found >= 3);
}

TEST_CASE(pipeline, scene_to_png_round_trip) {
    SceneDescription d = load_scene_file((kRoot / "scenes" / "glass-atrium.json").string());
    d.render.width = 48;
    d.render.height = 27;
    d.render.samples_per_pixel = 4;
    d.render.threads = 2;
    d.camera = Camera(d.camera_params, 48.0F / 27.0F);
    const Framebuffer fb = render(d.scene, d.camera, d.render);
    const auto cleaned = denoise(fb, {});
    const Image8 img = to_srgb8(cleaned, 48, 27, d.exposure);
    const auto decoded = decode_png(encode_png(img));
    REQUIRE(decoded.has_value());
    CHECK_EQ(decoded->width, 48U);
    CHECK(decoded->pixels == img.pixels);
    std::uint64_t sum = 0;
    for (const auto v : img.pixels) {
        sum += v;
    }
    CHECK(sum > 48U * 27U * 3U * 10U);  // not a black frame
}

TEST_CASE(hardening, numbers_that_overflow_single_precision_are_rejected_with_a_path) {
    const char* material = R"("materials": {"m": {}},)";
    for (const char* value : {"1e300", "-1e300", "1e39", "3.5e38", "1e10"}) {
        const std::string sphere_radius = std::string("{") + material + R"("objects": [{"type": "sphere", "center": [0,0,0], "radius": )" + value + R"(, "material": "m"}]})";
        CHECK(mentions(error_of(sphere_radius.c_str()), "objects[0].radius: magnitude"));
        const std::string center = std::string("{") + material + R"("objects": [{"type": "sphere", "center": [)" + value + R"(,0,0], "radius": 1, "material": "m"}]})";
        CHECK(mentions(error_of(center.c_str()), "objects[0].center[0]: magnitude"));
    }
    CHECK(mentions(error_of(R"({"camera": {"from": [1e300, 0, 0]}})"), "camera.from[0]: magnitude"));
    CHECK(mentions(error_of(R"({"settings": {"exposure": 1e300}})"), "settings.exposure: magnitude"));
    CHECK(mentions(error_of(R"({"materials": {"m": {"emission": [1e39, 0, 0]}}})"), "magnitude"));
    CHECK(mentions(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "mesh", "path": "../assets/models/rock.obj", "material": "m", "scale": 1e300}]})"),
                   "objects[0].scale: magnitude"));
    // The largest accepted values still produce a usable scene.
    CHECK(error_of(R"({"materials": {"m": {}}, "objects": [{"type": "sphere", "center": [1e9,0,0], "radius": 1e9, "material": "m"}]})").empty());
}

TEST_CASE(hardening, scene_rejects_non_finite_geometry_before_building_the_bvh) {
    Scene scene;
    const auto tex = scene.add_texture(ConstantTexture{Vec3(1)});
    Material m;
    m.base_texture = tex;
    const auto mat = scene.add_material(m);
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    CHECK_THROWS(scene.add_sphere({{kInf, 0, 0}, 1, mat}));
    CHECK_THROWS(scene.add_sphere({{nan, 0, 0}, 1, mat}));
    CHECK_THROWS(scene.add_sphere({{0, 0, 0}, kInf, mat}));
    CHECK_THROWS(scene.add_sphere({{0, 0, 0}, 3e38F, mat}));
    Triangle t;
    t.p0 = {0, 0, 0};
    t.p1 = {1, 0, 0};
    t.p2 = {0, nan, 0};
    t.material = mat;
    CHECK_THROWS(scene.add_triangle(t));
    t.p2 = {0, 1e30F, 0};
    CHECK_THROWS(scene.add_triangle(t));

    // OBJ values that fit a float but overflow once scaled.
    const ObjMesh huge = parse_obj("v 3e38 0 0\nv 0 3e38 0\nv 0 0 3e38\nf 1 2 3\n");
    CHECK_THROWS(scene.add_mesh(huge, Transform::from_trs({}, {}, {10, 10, 10}), mat, true));
    CHECK_THROWS(scene.add_mesh(huge, Transform::from_trs({}, {}, {1, 1, 1}), mat, true));
    CHECK_EQ(scene.primitive_count(), 0U);  // nothing half-added
    scene.build();  // an empty scene still builds
}

TEST_CASE(hardening, bvh_refuses_boxes_that_would_poison_the_bin_computation) {
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    CHECK_THROWS(Bvh::build(std::vector<Aabb>{{{0, 0, 0}, {1, 1, 1}}, {{nan, 0, 0}, {1, 1, 1}}}));
    CHECK_THROWS(Bvh::build(std::vector<Aabb>{{{0, 0, 0}, {1, 1, 1}}, {{-kInf, 0, 0}, {1, 1, 1}}}));
    CHECK_THROWS(Bvh::build(std::vector<Aabb>{Aabb{}}));  // default (empty, inverted) box
    CHECK_THROWS(Bvh::build(std::vector<Aabb>{{{-3e38F, 0, 0}, {3e38F, 1, 1}}}));  // extent overflows
}

TEST_CASE(hardening, oversized_files_are_refused_before_they_are_read) {
    const auto dir = std::filesystem::temp_directory_path();
    const auto path = dir / "lumen_oversized_scene.json";
    {
        std::ofstream create(path, std::ios::binary);
        create << "{}";
    }
    // A sparse file: 65 MiB of apparent size that costs (almost) no disk and must never be loaded.
    std::filesystem::resize_file(path, std::uintmax_t{65} << 20);
    std::string message;
    try {
        (void)load_scene_file(path.string());
    } catch (const InputError& e) {
        message = e.what();
    }
    std::filesystem::remove(path);
    CHECK(mentions(message, "file too large"));
    CHECK(mentions(message, "lumen_oversized_scene.json"));

    // The bounded reader enforces its limit exactly, and also when the size can't be queried up front.
    CHECK_EQ(read_file_bounded((kRoot / "scenes" / "knot-study.json").string(), 1 << 20).empty(), false);
    CHECK_THROWS(read_file_bounded((kRoot / "scenes" / "knot-study.json").string(), 100));
    CHECK_THROWS(read_file_bounded("/dev/zero", 4096));
    CHECK_THROWS(read_file_bounded("/nonexistent/file", 4096));
}
