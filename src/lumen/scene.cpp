#include "lumen/scene.hpp"

#include <algorithm>

#include "lumen/error.hpp"
#include "lumen/sampling.hpp"

namespace lumen {

Vec3 Environment::radiance(const Vec3& d) const {
    if (kind == Kind::Constant) {
        return horizon * intensity;
    }
    if (d.y >= 0) {
        // Smooth horizon-to-zenith ramp; the square root keeps the horizon band wide.
        return lerp(horizon, zenith, std::sqrt(clamp01(d.y))) * intensity;
    }
    return lerp(horizon, ground, std::sqrt(clamp01(-d.y))) * intensity;
}

namespace {

/// Largest coordinate accepted for geometry: the cross product of two edges, squared (as length() does), must
/// still fit in a float, which holds up to roughly 1e9.
constexpr Real kMaxCoordinate = Real(1e9);

void require_bounded(const Vec3& p, const char* what) {
    if (!(std::fabs(p.x) <= kMaxCoordinate && std::fabs(p.y) <= kMaxCoordinate && std::fabs(p.z) <= kMaxCoordinate)) {
        throw InputError(std::string(what) + " is not finite or exceeds 1e9 in magnitude");
    }
}

}  // namespace

std::uint32_t Scene::add_texture(Texture texture) {
    textures_.push_back(std::move(texture));
    return static_cast<std::uint32_t>(textures_.size() - 1);
}

std::uint32_t Scene::add_material(const Material& material) {
    if (material.base_texture >= textures_.size()) {
        throw InputError("material references a texture that does not exist");
    }
    materials_.push_back(material);
    return static_cast<std::uint32_t>(materials_.size() - 1);
}

void Scene::add_sphere(const Sphere& sphere) {
    if (sphere.material >= materials_.size()) {
        throw InputError("sphere references a material that does not exist");
    }
    if (!(sphere.radius > 0 && sphere.radius <= kMaxCoordinate)) {
        throw InputError("sphere radius must be positive and at most 1e9");
    }
    require_bounded(sphere.center, "sphere center");
    spheres_.push_back(sphere);
    prims_.push_back({PrimKind::Sphere, static_cast<std::uint32_t>(spheres_.size() - 1)});
}

bool Scene::add_triangle(const Triangle& triangle) {
    if (triangle.material >= materials_.size()) {
        throw InputError("triangle references a material that does not exist");
    }
    for (const Vec3* p : {&triangle.p0, &triangle.p1, &triangle.p2}) {
        require_bounded(*p, "triangle vertex");
    }
    if (!is_finite(triangle.n0) || !is_finite(triangle.n1) || !is_finite(triangle.n2)) {
        throw InputError("triangle normal is not finite");
    }
    if (!(triangle.area() > 1e-12F)) {
        return false;
    }
    triangles_.push_back(triangle);
    prims_.push_back({PrimKind::Triangle, static_cast<std::uint32_t>(triangles_.size() - 1)});
    return true;
}

std::size_t Scene::add_mesh(const ObjMesh& mesh, const Transform& transform, std::uint32_t material, bool smooth) {
    std::size_t added = 0;
    const bool flip = transform.flips_winding();
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        std::uint32_t idx[3] = {mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (flip) {
            std::swap(idx[1], idx[2]);
        }
        const ObjVertex* v[3] = {&mesh.vertices[idx[0]], &mesh.vertices[idx[1]], &mesh.vertices[idx[2]]};
        Triangle t;
        t.material = material;
        t.p0 = transform.point(v[0]->position);
        t.p1 = transform.point(v[1]->position);
        t.p2 = transform.point(v[2]->position);
        require_bounded(t.p0, "mesh vertex");  // transformed positions must stay representable
        require_bounded(t.p1, "mesh vertex");
        require_bounded(t.p2, "mesh vertex");
        if (!(t.area() > 1e-12F)) {
            continue;
        }
        const Vec3 face = t.face_normal();
        Vec3* normals[3] = {&t.n0, &t.n1, &t.n2};
        for (int k = 0; k < 3; ++k) {
            *normals[k] = (smooth && v[k]->has_normal) ? transform.normal(v[k]->normal) : face;
        }
        t.uv0 = v[0]->uv;
        t.uv1 = v[1]->uv;
        t.uv2 = v[2]->uv;
        add_triangle(t);
        ++added;
    }
    return added;
}

Aabb Scene::prim_bounds(const Prim& p) const {
    return p.kind == PrimKind::Sphere ? spheres_[p.index].bounds() : triangles_[p.index].bounds();
}

void Scene::build(Bvh::BuildOptions options) {
    std::vector<Aabb> boxes;
    boxes.reserve(prims_.size());
    bounds_ = Aabb{};
    for (const Prim& p : prims_) {
        boxes.push_back(prim_bounds(p));
        bounds_.grow(boxes.back());
    }
    bvh_ = Bvh::build(boxes, options);

    lights_.clear();
    light_of_prim_.assign(prims_.size(), -1);
    Real total = 0;
    for (std::uint32_t i = 0; i < prims_.size(); ++i) {
        const Prim& p = prims_[i];
        const std::uint32_t mat = p.kind == PrimKind::Sphere ? spheres_[p.index].material : triangles_[p.index].material;
        const Material& m = materials_[mat];
        if (!m.emissive()) {
            continue;
        }
        const Real area = p.kind == PrimKind::Sphere ? spheres_[p.index].area() : triangles_[p.index].area();
        const Real weight = area * luminance(m.emission);
        total += weight;
        light_of_prim_[i] = static_cast<std::int32_t>(lights_.size());
        lights_.push_back({i, weight, total});
    }
    for (Light& l : lights_) {
        l.cumulative /= total;
    }
    if (!lights_.empty()) {
        lights_.back().cumulative = 1;
    }
}

bool Scene::intersect_prim(std::uint32_t prim, const Ray& ray, Real& t_max, Real& u, Real& v) const {
    const Prim& p = prims_[prim];
    Real t = 0;
    if (p.kind == PrimKind::Sphere) {
        if (!spheres_[p.index].intersect(ray, t_max, t)) {
            return false;
        }
    } else if (!triangles_[p.index].intersect(ray, t_max, t, u, v)) {
        return false;
    }
    t_max = t;
    return true;
}

bool Scene::intersect(const Ray& ray, Real t_max, Hit& hit) const {
    std::uint32_t best = 0;
    Real bu = 0;
    Real bv = 0;
    const bool found = bvh_.traverse(ray, t_max, false, [&](std::uint32_t prim, Real& tm) {
        Real u = 0;
        Real v = 0;
        if (intersect_prim(prim, ray, tm, u, v)) {
            best = prim;
            bu = u;
            bv = v;
            return true;
        }
        return false;
    });
    if (found) {
        fill_hit(ray, best, t_max, bu, bv, hit);
    }
    return found;
}

bool Scene::occluded(const Ray& ray, Real t_max) const {
    return bvh_.traverse(ray, t_max, true, [&](std::uint32_t prim, Real& tm) {
        Real u = 0;
        Real v = 0;
        return intersect_prim(prim, ray, tm, u, v);
    });
}

void Scene::fill_hit(const Ray& ray, std::uint32_t prim, Real t, Real u, Real v, Hit& hit) const {
    const Prim& p = prims_[prim];
    hit.t = t;
    hit.prim = prim;
    hit.p = ray.at(t);
    Vec3 geo;
    Vec3 shade;
    if (p.kind == PrimKind::Sphere) {
        const Sphere& s = spheres_[p.index];
        geo = (hit.p - s.center) / s.radius;
        geo = normalize(geo);
        shade = geo;
        hit.uv = {Real(0.5) + std::atan2(geo.z, geo.x) * (kInvPi * Real(0.5)), Real(0.5) + std::asin(std::clamp(geo.y, Real(-1), Real(1))) * kInvPi};
        hit.material = s.material;
    } else {
        const Triangle& tri = triangles_[p.index];
        const Real w = Real(1) - u - v;
        geo = tri.face_normal();
        shade = normalize(tri.n0 * w + tri.n1 * u + tri.n2 * v);
        hit.uv = {tri.uv0.x * w + tri.uv1.x * u + tri.uv2.x * v, tri.uv0.y * w + tri.uv1.y * u + tri.uv2.y * v};
        hit.material = tri.material;
    }
    // Keep the shading normal in the geometric hemisphere (bad meshes can disagree).
    if (dot(shade, geo) < 0) {
        shade = -shade;
    }
    hit.front_face = dot(ray.direction, geo) < 0;
    hit.n_geo = hit.front_face ? geo : -geo;
    hit.n_shade = hit.front_face ? shade : -shade;
}

Bsdf Scene::bsdf_at(const Hit& hit) const {
    const Material& m = materials_[hit.material];
    if (m.kind == Material::Kind::Dielectric) {
        return Bsdf::dielectric(m.ior, m.tint, hit.front_face);
    }
    return Bsdf::standard(base_color(m, hit), m.metallic, m.roughness, m.specular);
}

Real Scene::selection_probability(std::int32_t light_index) const {
    const auto i = static_cast<std::size_t>(light_index);
    const Real previous = i == 0 ? Real(0) : lights_[i - 1].cumulative;
    return lights_[i].cumulative - previous;
}

std::optional<LightSample> Scene::sample_light(const Vec3& from, Real u_select, Vec2 u) const {
    if (lights_.empty()) {
        return std::nullopt;
    }
    const auto it = std::upper_bound(lights_.begin(), lights_.end(), u_select,
                                     [](Real value, const Light& l) { return value < l.cumulative; });
    const auto light_index = static_cast<std::int32_t>(std::min<std::size_t>(it - lights_.begin(), lights_.size() - 1));
    const Light& light = lights_[static_cast<std::size_t>(light_index)];
    const Prim& prim = prims_[light.prim];
    const Real select_prob = selection_probability(light_index);
    if (select_prob <= 0) {
        return std::nullopt;
    }

    LightSample out;
    if (prim.kind == PrimKind::Sphere) {
        const Sphere& s = spheres_[prim.index];
        out.radiance = materials_[s.material].emission;
        const Vec3 to_center = s.center - from;
        const Real d2 = length_squared(to_center);
        if (d2 > s.radius * s.radius) {
            const Real sin2_max = s.radius * s.radius / d2;
            const Real cos_max = safe_sqrt(Real(1) - sin2_max);
            const Onb frame = Onb::from_normal(normalize(to_center));
            const Vec3 wi = frame.to_world(sample_uniform_cone(u, cos_max));
            Real t = 0;
            if (!s.intersect({from, wi}, kInf, t)) {
                return std::nullopt;
            }
            out.wi = wi;
            out.distance = t;
            out.pdf = select_prob * uniform_cone_pdf(cos_max);
        } else {
            const Vec3 n = sample_uniform_sphere(u);
            const Vec3 point = s.center + n * s.radius;
            const Vec3 d = point - from;
            const Real dist2 = length_squared(d);
            if (dist2 <= 0) {
                return std::nullopt;
            }
            out.distance = std::sqrt(dist2);
            out.wi = d / out.distance;
            const Real cos_l = std::fabs(dot(n, out.wi));
            if (cos_l <= 1e-6F) {
                return std::nullopt;
            }
            out.pdf = select_prob * dist2 / (cos_l * s.area());
        }
    } else {
        const Triangle& t = triangles_[prim.index];
        out.radiance = materials_[t.material].emission;
        const Vec3 b = sample_uniform_triangle(u);
        const Vec3 point = t.p0 * b.x + t.p1 * b.y + t.p2 * b.z;
        const Vec3 d = point - from;
        const Real dist2 = length_squared(d);
        if (dist2 <= 0) {
            return std::nullopt;
        }
        out.distance = std::sqrt(dist2);
        out.wi = d / out.distance;
        const Real cos_l = std::fabs(dot(t.face_normal(), out.wi));
        if (cos_l <= 1e-6F) {
            return std::nullopt;
        }
        out.pdf = select_prob * dist2 / (cos_l * t.area());
    }
    return out;
}

Real Scene::light_pdf(const Vec3& from, const Hit& hit) const {
    const std::int32_t light_index = light_of_prim_[hit.prim];
    if (light_index < 0) {
        return 0;
    }
    const Prim& prim = prims_[hit.prim];
    const Real select_prob = selection_probability(light_index);
    if (prim.kind == PrimKind::Sphere) {
        const Sphere& s = spheres_[prim.index];
        const Real d2 = length_squared(s.center - from);
        if (d2 > s.radius * s.radius) {
            const Real cos_max = safe_sqrt(Real(1) - s.radius * s.radius / d2);
            return select_prob * uniform_cone_pdf(cos_max);
        }
        const Real cos_l = std::fabs(dot(hit.n_geo, normalize(hit.p - from)));
        return cos_l > 0 ? select_prob * length_squared(hit.p - from) / (cos_l * s.area()) : Real(0);
    }
    const Triangle& t = triangles_[prim.index];
    const Vec3 d = hit.p - from;
    const Real dist2 = length_squared(d);
    const Real cos_l = std::fabs(dot(t.face_normal(), d)) / std::sqrt(dist2);
    return cos_l > 0 ? select_prob * dist2 / (cos_l * t.area()) : Real(0);
}

}  // namespace lumen
