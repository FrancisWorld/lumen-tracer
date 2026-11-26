#include "lumen/scene_loader.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <set>

#include "lumen/error.hpp"
#include "lumen/fileio.hpp"

namespace lumen {
namespace {

constexpr double kMaxMagnitude = 1e9;

/// Typed, strict access to one JSON object. Tracks which keys were read.
class Fields {
public:
    Fields(const Json& value, std::string path) : path_(std::move(path)) {
        if (!value.is_object()) {
            throw InputError(path_ + ": expected an object but found " + kind_name(value.kind()));
        }
        object_ = &value;
    }

    const std::string& path() const { return path_; }

    [[noreturn]] void fail(const std::string& key, const std::string& what) const {
        throw InputError(path_ + "." + key + ": " + what);
    }

    bool has(const std::string& key) const { return object_->find(key) != nullptr; }

    const Json& require(const std::string& key) {
        const Json* v = object_->find(key);
        if (v == nullptr) {
            throw InputError(path_ + ": missing required key \"" + key + "\"");
        }
        used_.insert(key);
        return *v;
    }

    const Json* optional(const std::string& key) {
        const Json* v = object_->find(key);
        if (v != nullptr) {
            used_.insert(key);
        }
        return v;
    }

    double number(const std::string& key, double fallback) {
        const Json* v = optional(key);
        return v == nullptr ? fallback : as_number(*v, key);
    }
    double required_number(const std::string& key) { return as_number(require(key), key); }

    bool boolean(const std::string& key, bool fallback) {
        const Json* v = optional(key);
        if (v == nullptr) {
            return fallback;
        }
        if (!v->is_bool()) {
            fail(key, std::string("expected boolean but found ") + kind_name(v->kind()));
        }
        return v->as_bool();
    }

    std::string text(const std::string& key) {
        const Json& v = require(key);
        if (!v.is_string()) {
            fail(key, std::string("expected string but found ") + kind_name(v.kind()));
        }
        return v.as_string();
    }
    std::string text(const std::string& key, const std::string& fallback) {
        return has(key) ? text(key) : fallback;
    }

    Vec3 vec3(const std::string& key) { return as_vec3(require(key), key); }
    Vec3 vec3(const std::string& key, const Vec3& fallback) {
        const Json* v = optional(key);
        return v == nullptr ? fallback : as_vec3(*v, key);
    }

    Vec3 as_vec3(const Json& v, const std::string& key) const {
        if (!v.is_array() || v.items().size() != 3) {
            fail(key, "expected an array of three numbers");
        }
        Vec3 out;
        for (int i = 0; i < 3; ++i) {
            const Json& c = v.items()[static_cast<std::size_t>(i)];
            if (!c.is_number()) {
                fail(key + "[" + std::to_string(i) + "]", std::string("expected number but found ") + kind_name(c.kind()));
            }
            out[i] = static_cast<Real>(checked(c.as_number(), key + "[" + std::to_string(i) + "]"));
        }
        return out;
    }

    /// Every number must survive conversion to single precision and leave headroom for squares and
    /// cross products downstream, so |value| is capped at 1e9 (a finite double can still overflow a float, and edge cross products must stay finite).
    double checked(double v, const std::string& key) const {
        if (!(std::fabs(v) <= kMaxMagnitude)) {
            fail(key, "magnitude must not exceed 1e9");
        }
        return v;
    }


    /// Rejects keys that were never read.
    void finish() const {
        for (const auto& k : object_->keys()) {
            if (used_.count(k) == 0) {
                throw InputError(path_ + ": unknown key \"" + k + "\"");
            }
        }
    }

private:
    double as_number(const Json& v, const std::string& key) const {
        if (!v.is_number()) {
            fail(key, std::string("expected number but found ") + kind_name(v.kind()));
        }
        return checked(v.as_number(), key);
    }

    const Json* object_ = nullptr;
    std::string path_;
    std::set<std::string> used_;
};

Real positive(Fields& f, const std::string& key, double fallback) {
    const double v = f.number(key, fallback);
    if (!(v > 0)) {
        f.fail(key, "must be greater than zero");
    }
    return static_cast<Real>(v);
}

/// Whole number within [lo, hi]; avoids undefined float-to-integer conversions on hostile input.
std::int64_t integer(Fields& f, const std::string& key, std::int64_t fallback, std::int64_t lo, std::int64_t hi) {
    const double v = f.number(key, static_cast<double>(fallback));
    if (!(v >= static_cast<double>(lo) && v <= static_cast<double>(hi)) || v != std::floor(v)) {
        f.fail(key, "must be a whole number between " + std::to_string(lo) + " and " + std::to_string(hi));
    }
    return static_cast<std::int64_t>(v);
}

Real in_unit_range(Fields& f, const std::string& key, double fallback) {
    const double v = f.number(key, fallback);
    if (!(v >= 0 && v <= 1)) {
        f.fail(key, "must be between 0 and 1");
    }
    return static_cast<Real>(v);
}

Vec3 non_negative_color(Fields& f, const std::string& key, const Vec3& fallback) {
    const Vec3 c = f.vec3(key, fallback);
    if (min_component(c) < 0) {
        f.fail(key, "colour components must not be negative");
    }
    return c;
}

class Builder {
public:
    Builder(const Json& doc, std::filesystem::path base) : root_(doc, "scene"), base_(std::move(base)) {}

    SceneDescription build() {
        SceneDescription out;
        read_settings(out);
        read_camera(out);
        read_environment(out.scene);
        read_textures(out.scene);
        read_materials(out.scene);
        read_objects(out.scene);
        root_.finish();
        out.scene.build();
        return out;
    }

private:
    void read_settings(SceneDescription& out) {
        const Json* s = root_.optional("settings");
        RenderSettings& r = out.render;
        if (s == nullptr) {
            return;
        }
        Fields f(*s, "settings");
        r.width = static_cast<std::uint32_t>(integer(f, "width", r.width, 1, 16384));
        r.height = static_cast<std::uint32_t>(integer(f, "height", r.height, 1, 16384));
        r.samples_per_pixel = static_cast<std::uint32_t>(integer(f, "samples", r.samples_per_pixel, 1, 1'000'000));
        r.tile_size = static_cast<std::uint32_t>(integer(f, "tile_size", r.tile_size, 1, 1024));
        r.seed = static_cast<std::uint64_t>(integer(f, "seed", static_cast<std::int64_t>(r.seed), 0, 1LL << 52));
        r.path.max_depth = static_cast<int>(integer(f, "max_depth", r.path.max_depth, 0, 1000));
        r.path.roulette_depth = static_cast<int>(integer(f, "roulette_depth", r.path.roulette_depth, 0, 1000));
        r.path.sample_clamp = static_cast<Real>(f.number("sample_clamp", r.path.sample_clamp));
        out.exposure = positive(f, "exposure", 1);
        out.denoise = f.boolean("denoise", false);
        f.finish();
        r.validate();
    }

    void read_camera(SceneDescription& out) {
        CameraParams& p = out.camera_params;
        if (const Json* c = root_.optional("camera")) {
            Fields f(*c, "camera");
            p.look_from = f.vec3("from", p.look_from);
            p.look_at = f.vec3("at", p.look_at);
            p.up = f.vec3("up", p.up);
            p.vertical_fov_degrees = static_cast<Real>(f.number("fov", p.vertical_fov_degrees));
            p.aperture = static_cast<Real>(f.number("aperture", 0));
            p.focus_distance = static_cast<Real>(f.number("focus_distance", 0));
            f.finish();
        }
        out.camera = Camera(p, static_cast<Real>(out.render.width) / static_cast<Real>(out.render.height));
    }

    void read_environment(Scene& scene) {
        const Json* e = root_.optional("environment");
        if (e == nullptr) {
            return;
        }
        Fields f(*e, "environment");
        Environment env;
        const std::string type = f.text("type");
        env.intensity = static_cast<Real>(f.number("intensity", 1));
        if (env.intensity < 0) {
            f.fail("intensity", "must not be negative");
        }
        if (type == "constant") {
            env.kind = Environment::Kind::Constant;
            env.horizon = non_negative_color(f, "color", Vec3(0));
        } else if (type == "gradient") {
            env.kind = Environment::Kind::Gradient;
            env.horizon = non_negative_color(f, "horizon", Vec3(1));
            env.zenith = non_negative_color(f, "zenith", Vec3(0.5F, 0.7F, 1));
            env.ground = non_negative_color(f, "ground", env.horizon * 0.5F);
        } else {
            f.fail("type", "unknown environment \"" + type + "\" (expected constant or gradient)");
        }
        f.finish();
        scene.set_environment(env);
    }

    void read_textures(Scene& scene) {
        const Json* t = root_.optional("textures");
        if (t == nullptr) {
            return;
        }
        if (!t->is_object()) {
            throw InputError("textures: expected an object");
        }
        for (std::size_t i = 0; i < t->keys().size(); ++i) {
            const std::string& name = t->keys()[i];
            Fields f(t->values()[i], "textures." + name);
            const std::string type = f.text("type");
            Texture tex;
            if (type == "constant") {
                tex = ConstantTexture{non_negative_color(f, "color", Vec3(0.5F))};
            } else if (type == "checker") {
                tex = CheckerTexture{non_negative_color(f, "a", Vec3(0.9F)), non_negative_color(f, "b", Vec3(0.1F)),
                                     positive(f, "scale", 8)};
            } else if (type == "image") {
                const std::string rel = f.text("path");
                const auto img = read_png_file((base_ / rel).string());
                if (!img) {
                    f.fail("path", rel + ": " + img.error());
                }
                Vec2 repeat{1, 1};
                if (const Json* r = f.optional("repeat")) {
                    if (!r->is_array() || r->items().size() != 2 || !r->items()[0].is_number() || !r->items()[1].is_number()) {
                        f.fail("repeat", "expected an array of two numbers");
                    }
                    repeat = {static_cast<Real>(f.checked(r->items()[0].as_number(), "repeat[0]")),
                              static_cast<Real>(f.checked(r->items()[1].as_number(), "repeat[1]"))};
                    if (!(repeat.x > 0 && repeat.y > 0)) {
                        f.fail("repeat", "components must be greater than zero");
                    }
                }
                tex = ImageTexture::from_image(*img, f.boolean("srgb", true), repeat);
            } else {
                f.fail("type", "unknown texture \"" + type + "\" (expected constant, checker or image)");
            }
            f.finish();
            textures_[name] = scene.add_texture(std::move(tex));
        }
    }

    std::uint32_t colour_to_texture(Scene& scene, Fields& f, const std::string& key, const Vec3& fallback) {
        const Json* v = f.optional(key);
        if (v == nullptr) {
            return scene.add_texture(ConstantTexture{fallback});
        }
        if (v->is_string()) {
            const auto it = textures_.find(v->as_string());
            if (it == textures_.end()) {
                f.fail(key, "unknown texture \"" + v->as_string() + "\"");
            }
            return it->second;
        }
        const Vec3 c = f.as_vec3(*v, key);
        if (min_component(c) < 0) {
            f.fail(key, "colour components must not be negative");
        }
        return scene.add_texture(ConstantTexture{c});
    }

    void read_materials(Scene& scene) {
        const Json* m = root_.optional("materials");
        if (m == nullptr) {
            return;
        }
        if (!m->is_object()) {
            throw InputError("materials: expected an object");
        }
        for (std::size_t i = 0; i < m->keys().size(); ++i) {
            const std::string& name = m->keys()[i];
            Fields f(m->values()[i], "materials." + name);
            Material mat;
            const std::string type = f.text("type", "standard");
            if (type == "standard") {
                mat.base_texture = colour_to_texture(scene, f, "base_color", Vec3(0.8F));
                mat.metallic = in_unit_range(f, "metallic", 0);
                mat.roughness = in_unit_range(f, "roughness", 0.5);
                mat.specular = in_unit_range(f, "specular", 0.5);
                mat.emission = non_negative_color(f, "emission", Vec3(0)) * positive(f, "emission_strength", 1);
            } else if (type == "dielectric") {
                mat.kind = Material::Kind::Dielectric;
                mat.base_texture = scene.add_texture(ConstantTexture{Vec3(1)});
                mat.ior = static_cast<Real>(f.number("ior", 1.5));
                if (!(mat.ior >= 1 && mat.ior <= 4)) {
                    f.fail("ior", "must be between 1 and 4");
                }
                mat.tint = non_negative_color(f, "tint", Vec3(1));
                if (max_component(mat.tint) > 1) {
                    f.fail("tint", "components must not exceed 1 (glass cannot amplify light)");
                }
            } else {
                f.fail("type", "unknown material \"" + type + "\" (expected standard or dielectric)");
            }
            f.finish();
            materials_[name] = scene.add_material(mat);
        }
    }

    std::uint32_t material_of(Scene& scene, Fields& f) {
        const std::string name = f.text("material");
        const auto it = materials_.find(name);
        if (it == materials_.end()) {
            f.fail("material", "unknown material \"" + name + "\"");
        }
        (void)scene;
        return it->second;
    }

    void read_objects(Scene& scene) {
        const Json* o = root_.optional("objects");
        if (o == nullptr) {
            return;
        }
        if (!o->is_array()) {
            throw InputError("objects: expected an array");
        }
        for (std::size_t i = 0; i < o->items().size(); ++i) {
            Fields f(o->items()[i], "objects[" + std::to_string(i) + "]");
            const std::string type = f.text("type");
            if (type == "sphere") {
                Sphere s;
                s.center = f.vec3("center");
                s.radius = positive(f, "radius", 1);
                s.material = material_of(scene, f);
                scene.add_sphere(s);
            } else if (type == "triangle") {
                add_triangle(scene, f);
            } else if (type == "quad") {
                add_quad(scene, f);
            } else if (type == "mesh") {
                add_mesh(scene, f);
            } else {
                f.fail("type", "unknown object \"" + type + "\" (expected sphere, triangle, quad or mesh)");
            }
            f.finish();
        }
    }

    void add_triangle(Scene& scene, Fields& f) {
        const Json& verts = f.require("vertices");
        if (!verts.is_array() || verts.items().size() != 3) {
            f.fail("vertices", "expected an array of three points");
        }
        Triangle t;
        t.p0 = f.as_vec3(verts.items()[0], "vertices[0]");
        t.p1 = f.as_vec3(verts.items()[1], "vertices[1]");
        t.p2 = f.as_vec3(verts.items()[2], "vertices[2]");
        t.material = material_of(scene, f);
        t.uv0 = {0, 0};
        t.uv1 = {1, 0};
        t.uv2 = {0, 1};
        if (!(t.area() > 1e-12F)) {
            f.fail("vertices", "triangle has zero area");
        }
        t.n0 = t.n1 = t.n2 = t.face_normal();
        scene.add_triangle(t);
    }

    void add_quad(Scene& scene, Fields& f) {
        const Vec3 corner = f.vec3("corner");
        const Vec3 u = f.vec3("u");
        const Vec3 v = f.vec3("v");
        const std::uint32_t material = material_of(scene, f);
        if (!(length(cross(u, v)) > 1e-9F)) {
            f.fail("u", "u and v must span a non-degenerate parallelogram");
        }
        const Vec3 p[4] = {corner, corner + u, corner + u + v, corner + v};
        const Vec2 uv[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        for (const auto& idx : {std::array<int, 3>{0, 1, 2}, std::array<int, 3>{0, 2, 3}}) {
            Triangle t;
            t.p0 = p[idx[0]];
            t.p1 = p[idx[1]];
            t.p2 = p[idx[2]];
            t.uv0 = uv[idx[0]];
            t.uv1 = uv[idx[1]];
            t.uv2 = uv[idx[2]];
            t.n0 = t.n1 = t.n2 = t.face_normal();
            t.material = material;
            scene.add_triangle(t);
        }
    }

    void add_mesh(Scene& scene, Fields& f) {
        const std::string rel = f.text("path");
        const std::uint32_t material = material_of(scene, f);
        Vec3 scale(1);
        if (const Json* s = f.optional("scale")) {
            scale = s->is_number() ? Vec3(static_cast<Real>(f.checked(s->as_number(), "scale"))) : f.as_vec3(*s, "scale");
        }
        if (!(scale.x != 0 && scale.y != 0 && scale.z != 0)) {
            f.fail("scale", "must not be zero");
        }
        const Transform tr = Transform::from_trs(f.vec3("translate", Vec3(0)), f.vec3("rotate", Vec3(0)), scale);
        const bool smooth = f.boolean("smooth", true);
        const ObjMesh mesh = load_obj_file((base_ / rel).string());
        if (scene.add_mesh(mesh, tr, material, smooth) == 0) {
            f.fail("path", rel + ": mesh contains no usable triangles");
        }
    }

    Fields root_;
    std::filesystem::path base_;
    std::map<std::string, std::uint32_t> textures_;
    std::map<std::string, std::uint32_t> materials_;
};

}  // namespace

SceneDescription build_scene(const Json& document, const std::filesystem::path& base_dir) {
    return Builder(document, base_dir).build();
}

SceneDescription load_scene_file(const std::string& path) {
    const std::filesystem::path p(path);
    Json doc;
    try {
        doc = parse_json(read_file_bounded(p.string(), kMaxSceneBytes));
        return build_scene(doc, p.parent_path());
    } catch (const InputError& e) {
        throw InputError(p.filename().string() + ": " + e.what());
    }
}

}  // namespace lumen
