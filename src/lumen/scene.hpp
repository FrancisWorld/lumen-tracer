// Scene container: primitives, materials, textures, environment, BVH and the
// light list used for next-event estimation.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "lumen/bsdf.hpp"
#include "lumen/bvh.hpp"
#include "lumen/geometry.hpp"
#include "lumen/obj.hpp"
#include "lumen/texture.hpp"

namespace lumen {

struct Material {
    enum class Kind { Standard, Dielectric };
    Kind kind = Kind::Standard;
    std::uint32_t base_texture = 0;  // index into the scene's textures
    Real metallic = 0;
    Real roughness = 0.5F;
    Real specular = 0.5F;
    Vec3 emission{};  // radiance emitted from both sides
    Real ior = 1.5F;
    Vec3 tint{1, 1, 1};

    bool emissive() const { return max_component(emission) > 0; }
};

struct Environment {
    enum class Kind { Constant, Gradient };
    Kind kind = Kind::Constant;
    Vec3 horizon{0, 0, 0};  // also the colour of the Constant sky
    Vec3 zenith{0, 0, 0};
    Vec3 ground{0, 0, 0};
    Real intensity = 1;

    /// Radiance arriving from direction `d` (unit length, +y is up).
    Vec3 radiance(const Vec3& d) const;
};

enum class PrimKind : std::uint8_t { Sphere, Triangle };

struct Hit {
    Real t = kInf;
    Vec3 p;
    Vec3 n_geo;    // geometric normal, flipped to face the incoming ray
    Vec3 n_shade;  // interpolated normal on the same side as n_geo
    Vec2 uv;
    bool front_face = true;  // ray hit the side the outward normal points to
    std::uint32_t material = 0;
    std::uint32_t prim = 0;  // index into the scene's primitive list
};

struct LightSample {
    Vec3 wi;
    Real distance = 0;
    Real pdf = 0;  // solid angle, includes the probability of choosing this light
    Vec3 radiance;
};

class Scene {
public:
    std::uint32_t add_texture(Texture texture);
    std::uint32_t add_material(const Material& material);
    void add_sphere(const Sphere& sphere);
    /// Returns false (and adds nothing) for zero-area triangles.
    bool add_triangle(const Triangle& triangle);
    /// Adds an OBJ mesh through `transform`. Returns the number of triangles added.
    std::size_t add_mesh(const ObjMesh& mesh, const Transform& transform, std::uint32_t material, bool smooth);
    void set_environment(const Environment& env) { environment_ = env; }

    /// Builds the BVH and light distribution. Must be called after the last add_*.
    void build(Bvh::BuildOptions options = {});

    bool intersect(const Ray& ray, Real t_max, Hit& hit) const;
    bool occluded(const Ray& ray, Real t_max) const;
    Vec3 environment(const Vec3& direction) const { return environment_.radiance(direction); }

    const Material& material(std::uint32_t id) const { return materials_[id]; }
    Vec3 base_color(const Material& m, const Hit& hit) const { return evaluate(textures_[m.base_texture], hit.uv); }
    Bsdf bsdf_at(const Hit& hit) const;

    std::size_t primitive_count() const { return prims_.size(); }
    std::size_t light_count() const { return lights_.size(); }
    const Bvh& bvh() const { return bvh_; }
    const Aabb& bounds() const { return bounds_; }

    /// Pick a light and a point on it as seen from `from`.
    std::optional<LightSample> sample_light(const Vec3& from, Real u_select, Vec2 u) const;
    /// Solid-angle density with which sample_light() would have produced `hit`, seen from `from`.
    Real light_pdf(const Vec3& from, const Hit& hit) const;

private:
    struct Prim {
        PrimKind kind;
        std::uint32_t index;
    };
    struct Light {
        std::uint32_t prim;
        Real weight;       // unnormalised selection weight (emitted power)
        Real cumulative;   // running sum, normalised to 1 after build()
    };

    Aabb prim_bounds(const Prim& p) const;
    bool intersect_prim(std::uint32_t prim, const Ray& ray, Real& t_max, Real& u, Real& v) const;
    void fill_hit(const Ray& ray, std::uint32_t prim, Real t, Real u, Real v, Hit& hit) const;
    Real selection_probability(std::int32_t light_index) const;

    std::vector<Texture> textures_;
    std::vector<Material> materials_;
    std::vector<Sphere> spheres_;
    std::vector<Triangle> triangles_;
    std::vector<Prim> prims_;
    std::vector<std::int32_t> light_of_prim_;
    std::vector<Light> lights_;
    Environment environment_;
    Bvh bvh_;
    Aabb bounds_;
};

}  // namespace lumen
