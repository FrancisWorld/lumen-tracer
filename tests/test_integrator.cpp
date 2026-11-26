#include "lumen/integrator.hpp"
#include <array>

#include "testing.hpp"

using namespace lumen;

namespace {

struct World {
    Scene scene;
    std::uint32_t texture = 0;
    World() { texture = scene.add_texture(ConstantTexture{Vec3(1)}); }

    std::uint32_t diffuse(Real albedo, Real emission = 0) {
        Material m;
        m.base_texture = scene.add_texture(ConstantTexture{Vec3(albedo)});
        m.specular = 0;  // pure Lambertian: closed-form answers exist
        m.emission = Vec3(emission);
        return scene.add_material(m);
    }
    std::uint32_t glass() {
        Material m;
        m.kind = Material::Kind::Dielectric;
        m.base_texture = texture;
        return scene.add_material(m);
    }
    void sky(Real radiance) {
        Environment e;
        e.horizon = Vec3(radiance);
        scene.set_environment(e);
    }
    /// Large horizontal square at y = 0 facing up.
    void floor(std::uint32_t material) {
        const Vec3 p[4] = {{-50, 0, -50}, {-50, 0, 50}, {50, 0, 50}, {50, 0, -50}};
        for (const auto& tri : {std::array<int, 3>{0, 1, 3}, std::array<int, 3>{2, 3, 1}}) {
            Triangle t;
            t.p0 = p[tri[0]];
            t.p1 = p[tri[1]];
            t.p2 = p[tri[2]];
            t.n0 = t.n1 = t.n2 = Vec3(0, 1, 0);
            t.material = material;
            scene.add_triangle(t);
        }
    }
    void box(Real half, std::uint32_t material) {
        const Vec3 c[8] = {{-half, -half, -half}, {half, -half, -half}, {half, half, -half}, {-half, half, -half},
                           {-half, -half, half},  {half, -half, half},  {half, half, half},  {-half, half, half}};
        const int quads[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 2, 6, 7}, {0, 3, 7, 4}, {1, 2, 6, 5}};
        for (const auto& q : quads) {
            for (const auto& tri : {std::array<int, 3>{q[0], q[1], q[2]}, std::array<int, 3>{q[0], q[2], q[3]}}) {
                Triangle t;
                t.p0 = c[tri[0]];
                t.p1 = c[tri[1]];
                t.p2 = c[tri[2]];
                t.n0 = t.n1 = t.n2 = t.face_normal();
                t.material = material;
                scene.add_triangle(t);
            }
        }
    }
};

Vec3 mean_radiance(const World& w, const Ray& ray, int samples, const PathSettings& settings, std::uint64_t seed = 1) {
    Pcg32 rng(seed);
    Vec3 sum;
    for (int i = 0; i < samples; ++i) {
        sum += trace_path(w.scene, ray, rng, settings).radiance;
    }
    return sum / static_cast<Real>(samples);
}

}  // namespace

TEST_CASE(furnace, lambertian_sphere_returns_its_albedo) {
    World w;
    w.sky(1);
    w.scene.add_sphere({{0, 0, 0}, 1, w.diffuse(0.7F)});
    w.scene.build();
    const Vec3 l = mean_radiance(w, {{0, 0, 4}, {0, 0, -1}}, 500, {.max_depth = 16});
    CHECK_NEAR(l.x, 0.7, 1e-4);  // convex object: one bounce and the ray escapes into a uniform sky
    CHECK_NEAR(l.z, 0.7, 1e-4);
}

TEST_CASE(furnace, rough_dielectric_coat_never_creates_energy) {
    for (const Real rough : {0.1F, 0.5F, 1.0F}) {
        for (const Real metallic : {0.0F, 1.0F}) {
            World w;
            w.sky(1);
            Material m;
            m.base_texture = w.texture;  // white
            m.roughness = rough;
            m.metallic = metallic;
            w.scene.add_sphere({{0, 0, 0}, 1, w.scene.add_material(m)});
            w.scene.build();
            const Real l = mean_radiance(w, {{0, 0, 4}, {0, 0, -1}}, 20000, {.max_depth = 16}).x;
            CHECK(l <= 1.01F);
            // Single-scatter GGX loses energy as roughness grows (about 70% at alpha = 1 for a pure metal).
            CHECK(l > (rough >= 1.0F && metallic > 0 ? 0.25F : 0.8F));
        }
    }
}

TEST_CASE(furnace, glass_sphere_is_lossless) {
    World w;
    w.sky(1);
    w.scene.add_sphere({{0, 0, 0}, 1, w.glass()});
    w.scene.build();
    const Vec3 l = mean_radiance(w, {{0.3F, 0.2F, 4}, {0, 0, -1}}, 20000, {.max_depth = 64, .roulette_depth = 64});
    CHECK_NEAR(l.x, 1.0, 0.01);
    // Hollow glass shell (two nested spheres) must conserve energy too.
    World shell;
    shell.sky(1);
    const auto g = shell.glass();
    shell.scene.add_sphere({{0, 0, 0}, 1, g});
    shell.scene.add_sphere({{0, 0, 0}, 0.8F, g});
    shell.scene.build();
    CHECK_NEAR(mean_radiance(shell, {{0.1F, 0.1F, 4}, {0, 0, -1}}, 20000, {.max_depth = 64, .roulette_depth = 64}).x, 1.0, 0.02);
}

TEST_CASE(furnace, emissive_sphere_cavity_reaches_radiosity_solution) {
    // Closed sphere with emission E and albedo a: L = E / (1 - a) everywhere. 0.5 / (1 - 0.5) = 1.
    for (const int roulette : {2, 1000}) {  // roulette must not bias the result
        World w;
        w.scene.add_sphere({{0, 0, 0}, 5, w.diffuse(0.5F, 0.5F)});
        w.scene.build();
        const Vec3 l = mean_radiance(w, {{0.5F, 0, 0}, {0, 0, -1}}, 40000, {.max_depth = 64, .roulette_depth = roulette});
        CHECK_NEAR(l.x, 1.0, 0.02);
    }
}

TEST_CASE(furnace, emissive_triangle_box_cavity_reaches_radiosity_solution) {
    // Same identity with twelve emissive triangles: exercises triangle light sampling and MIS from inside.
    World w;
    w.box(3, w.diffuse(0.6F, 0.4F));  // 0.4 / (1 - 0.6) = 1
    w.scene.build();
    CHECK_EQ(w.scene.light_count(), 12U);
    const Vec3 l = mean_radiance(w, {{0.2F, -0.3F, 0.1F}, normalize(Vec3(0.3F, 0.1F, -1))}, 60000,
                                 {.max_depth = 64, .roulette_depth = 4});
    CHECK_NEAR(l.x, 1.0, 0.025);
}

TEST_CASE(direct, matches_analytic_irradiance_of_a_spherical_light) {
    // Lambertian floor point under a sphere light: L_o = rho * L_e * (r/d)^2 * cos(theta).
    World w;
    w.floor(w.diffuse(0.5F));
    w.scene.add_sphere({{0, 3, 0}, 0.5F, w.diffuse(0.1F, 20)});
    w.scene.build();
    const PathSettings direct_only{.max_depth = 1, .roulette_depth = 8};
    const double r2 = 0.25;
    for (const Vec3 target : {Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(-1, 0, 4)}) {
        const Vec3 light(0, 3, 0);
        const Vec3 d = light - target;
        const double d2 = length_squared(d);
        const double cos_theta = d.y / std::sqrt(d2);
        const double expected = 0.5 * 20 * (r2 / d2) * cos_theta;
        const Vec3 origin = target + Vec3(6, 0.5F, 0.3F);
        const Ray ray{origin, normalize(target - origin)};
        const Real got = mean_radiance(w, ray, 30000, direct_only).x;
        CHECK_NEAR(got, expected, expected * 0.03);
    }
}

TEST_CASE(direct, occluders_cast_shadows) {
    World w;
    w.floor(w.diffuse(0.5F));
    w.scene.add_sphere({{0, 3, 0}, 0.5F, w.diffuse(0.1F, 20)});
    w.scene.add_sphere({{0, 1.5F, 0}, 0.7F, w.diffuse(0.5F)});  // blocker between lamp and floor point
    w.scene.build();
    const Ray ray{{6, 0.5F, 0.3F}, normalize(Vec3(-6, -0.5F, -0.3F))};
    CHECK(mean_radiance(w, ray, 5000, {.max_depth = 1}).x < 1e-6F);
    // Moving the aim point out of the shadow restores light.
    const Ray lit{{6, 0.5F, 0.3F}, normalize(Vec3(-6 + 4, -0.5F, -0.3F))};
    CHECK(mean_radiance(w, lit, 5000, {.max_depth = 1}).x > 0.01F);
}

TEST_CASE(path, emission_and_sky_are_seen_directly) {
    World w;
    w.sky(0.25F);
    w.scene.add_sphere({{0, 0, -5}, 1, w.diffuse(0.5F, 3)});
    w.scene.build();
    Pcg32 rng(1);
    const PathSample hit = trace_path(w.scene, {{0, 0, 0}, {0, 0, -1}}, rng, {.max_depth = 0});
    CHECK_NEAR(hit.radiance.x, 3.0, 1e-5);
    const PathSample miss = trace_path(w.scene, {{0, 0, 0}, {0, 1, 0}}, rng, {.max_depth = 0});
    CHECK_NEAR(miss.radiance.x, 0.25, 1e-6);
    CHECK(std::isinf(miss.guides.depth));
}

TEST_CASE(path, guides_come_from_first_non_specular_surface) {
    World w;
    w.sky(1);
    Material m;
    m.base_texture = w.scene.add_texture(ConstantTexture{{0.2F, 0.4F, 0.6F}});
    w.scene.add_sphere({{0, 0, -10}, 2, w.scene.add_material(m)});
    w.scene.add_sphere({{0, 0, -4}, 0.5F, w.glass()});  // a glass bead in front
    w.scene.build();
    Pcg32 rng(1);
    const PathSample s = trace_path(w.scene, {{0, 0, 0}, {0, 0, -1}}, rng, {.max_depth = 8});
    CHECK_NEAR(s.guides.albedo.y, 0.4, 1e-5);
    CHECK_NEAR(s.guides.normal.z, 1.0, 0.05);  // the glass is skipped, the sphere behind is described
    CHECK(s.guides.depth > 7.5F && s.guides.depth < 9.0F);
}

TEST_CASE(path, sample_clamp_bounds_fireflies_and_runs_are_reproducible) {
    World w;
    w.sky(1);
    w.scene.add_sphere({{0, 0, -5}, 1, w.diffuse(0.5F, 500)});
    w.scene.build();
    Pcg32 a(9);
    Pcg32 b(9);
    const Ray ray{{0, 0, 0}, {0, 0, -1}};
    const PathSample clamped = trace_path(w.scene, ray, a, {.sample_clamp = 10});
    CHECK_NEAR(max_component(clamped.radiance), 10.0, 1e-4);
    const PathSample again = trace_path(w.scene, ray, b, {.sample_clamp = 10});
    CHECK_NEAR(clamped.radiance.x, again.radiance.x, 0);
}

TEST_CASE(path, ray_origin_offset_leaves_the_surface_on_the_right_side) {
    Hit hit;
    hit.p = {10, 0, 0};
    hit.n_geo = {0, 1, 0};
    CHECK(offset_ray_origin(hit, {0, 1, 0}).y > 0);
    CHECK(offset_ray_origin(hit, {0, -1, 0}).y < 0);
    CHECK(offset_ray_origin(hit, {1, 1, 0}).y < 1e-2F);  // a nudge, not a jump
}
