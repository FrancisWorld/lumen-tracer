#include "lumen/rng.hpp"
#include "lumen/scene.hpp"
#include "testing.hpp"

using namespace lumen;

namespace {

struct Fixture {
    Scene scene;
    std::uint32_t grey = 0;
    std::uint32_t lamp = 0;
    Fixture() {
        const auto tex = scene.add_texture(ConstantTexture{Vec3(0.5F)});
        Material m;
        m.base_texture = tex;
        grey = scene.add_material(m);
        Material e;
        e.base_texture = tex;
        e.emission = Vec3(4, 4, 4);
        lamp = scene.add_material(e);
    }
};

/// Solid angle of a planar triangle seen from `p` (Van Oosterom & Strackee).
double triangle_solid_angle(const Vec3& p, const Triangle& t) {
    const Vec3 a = normalize(t.p0 - p);
    const Vec3 b = normalize(t.p1 - p);
    const Vec3 c = normalize(t.p2 - p);
    const double num = std::fabs(dot(a, cross(b, c)));
    const double den = 1.0 + dot(a, b) + dot(b, c) + dot(c, a);
    return 2.0 * std::atan2(num, den);
}

}  // namespace

TEST_CASE(scene, sphere_hits_report_normals_uv_and_side) {
    Fixture f;
    f.scene.add_sphere({{0, 0, -5}, 1, f.grey});
    f.scene.build();
    Hit hit;
    REQUIRE(f.scene.intersect({{0, 0, 0}, {0, 0, -1}}, kInf, hit));
    CHECK_NEAR(hit.t, 4.0, 1e-4);
    CHECK_NEAR(hit.n_geo.z, 1.0, 1e-4);  // faces the viewer
    CHECK(hit.front_face);
    CHECK_NEAR(hit.uv.y, 0.5, 1e-3);     // equator
    // From inside, the stored normal flips to face the ray and front_face is false.
    Hit inside;
    REQUIRE(f.scene.intersect({{0, 0, -5}, {0, 0, -1}}, kInf, inside));
    CHECK(!inside.front_face);
    CHECK_NEAR(dot(inside.n_geo, Vec3(0, 0, -1)), -1.0, 1e-4);
    CHECK(!f.scene.intersect({{0, 3, 0}, {0, 0, -1}}, kInf, hit));
    CHECK(!f.scene.intersect({{0, 0, 0}, {0, 0, -1}}, 3.0F, hit));  // t_max honoured
}

TEST_CASE(scene, triangle_hits_interpolate_normals_and_uv) {
    Fixture f;
    Triangle t;
    t.p0 = {-1, -1, -2};
    t.p1 = {1, -1, -2};
    t.p2 = {0, 1, -2};
    t.n0 = normalize(Vec3(-0.3F, 0, 1));
    t.n1 = normalize(Vec3(0.3F, 0, 1));
    t.n2 = Vec3(0, 0, 1);
    t.uv0 = {0, 0};
    t.uv1 = {1, 0};
    t.uv2 = {0.5F, 1};
    t.material = f.grey;
    REQUIRE(f.scene.add_triangle(t));
    f.scene.build();
    Hit hit;
    REQUIRE(f.scene.intersect({{0.5F, -0.5F, 0}, {0, 0, -1}}, kInf, hit));
    CHECK_NEAR(hit.p.z, -2.0, 1e-5);
    CHECK(hit.n_shade.x > 0.05F);  // pulled toward the +x vertex normal
    CHECK_NEAR(length(hit.n_shade), 1.0, 1e-5);
    CHECK(hit.uv.x > 0.5F && hit.uv.x < 1.0F);
    // Seen from behind the triangle the normals flip together.
    Hit back;
    REQUIRE(f.scene.intersect({{0.5F, -0.5F, -4}, {0, 0, 1}}, kInf, back));
    CHECK(!back.front_face);
    CHECK(dot(back.n_geo, back.n_shade) > 0);
}

TEST_CASE(scene, degenerate_triangles_and_invalid_references_are_rejected) {
    Fixture f;
    Triangle flat;
    flat.p0 = {0, 0, 0};
    flat.p1 = {1, 0, 0};
    flat.p2 = {2, 0, 0};  // collinear
    flat.material = f.grey;
    CHECK(!f.scene.add_triangle(flat));
    CHECK_THROWS(f.scene.add_sphere({{0, 0, 0}, 1, 99}));
    CHECK_THROWS(f.scene.add_sphere({{0, 0, 0}, 0, f.grey}));
    Material bad;
    bad.base_texture = 42;
    CHECK_THROWS(f.scene.add_material(bad));
    f.scene.build();
    CHECK_EQ(f.scene.primitive_count(), 0U);
    Hit hit;
    CHECK(!f.scene.intersect({{0, 0, 0}, {0, 0, 1}}, kInf, hit));  // empty scene is safe
}

TEST_CASE(scene, occlusion_queries_respect_distance) {
    Fixture f;
    f.scene.add_sphere({{0, 0, -5}, 1, f.grey});
    f.scene.build();
    const Ray ray{{0, 0, 0}, {0, 0, -1}};
    CHECK(f.scene.occluded(ray, 10));
    CHECK(!f.scene.occluded(ray, 3.5F));
    CHECK(!f.scene.occluded({{0, 3, 0}, {0, 0, -1}}, 10));
}

TEST_CASE(scene, mesh_transform_places_and_mirrors_geometry) {
    Fixture f;
    const ObjMesh tri = parse_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    CHECK_EQ(f.scene.add_mesh(tri, Transform::from_trs({0, 0, -3}, {}, {2, 2, 2}), f.grey, true), 1U);
    // A mirrored copy must still face the same way after its winding is repaired.
    CHECK_EQ(f.scene.add_mesh(tri, Transform::from_trs({5, 0, -3}, {}, {-2, 2, 2}), f.grey, true), 1U);
    f.scene.build();
    Hit a;
    REQUIRE(f.scene.intersect({{0.5F, 0.5F, 0}, {0, 0, -1}}, kInf, a));
    CHECK_NEAR(a.t, 3.0, 1e-4);
    CHECK(a.front_face);
    Hit b;
    REQUIRE(f.scene.intersect({{4.0F, 0.5F, 0}, {0, 0, -1}}, kInf, b));
    CHECK(b.front_face);
    CHECK_NEAR(b.n_geo.z, 1.0, 1e-4);
}

TEST_CASE(scene, environment_gradient_and_constant) {
    Environment c;
    c.horizon = {0.2F, 0.3F, 0.4F};
    c.intensity = 2;
    CHECK_NEAR(c.radiance({0, 1, 0}).y, 0.6, 1e-6);
    Environment g;
    g.kind = Environment::Kind::Gradient;
    g.horizon = Vec3(1);
    g.zenith = Vec3(0, 0, 1);
    g.ground = Vec3(0.5F);
    CHECK_NEAR(g.radiance({1, 0, 0}).x, 1.0, 1e-6);
    CHECK_NEAR(g.radiance({0, 1, 0}).x, 0.0, 1e-6);
    CHECK_NEAR(g.radiance({0, -1, 0}).x, 0.5, 1e-6);
}

TEST_CASE(light, sphere_light_pdf_integrates_to_solid_angle_from_outside) {
    Fixture f;
    f.scene.add_sphere({{0, 0, -6}, 1.5F, f.lamp});
    f.scene.build();
    CHECK_EQ(f.scene.light_count(), 1U);
    const Vec3 from(0.3F, -0.2F, 0);
    const Real d = length(Vec3(0, 0, -6) - from);
    const double expected = 2 * std::numbers::pi * (1 - std::sqrt(1 - 1.5 * 1.5 / (d * d)));
    Pcg32 rng(1);
    double estimate = 0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const auto s = f.scene.sample_light(from, rng.next_real(), rng.next_vec2());
        REQUIRE(s.has_value());
        estimate += 1.0 / s->pdf;  // E[1/pdf] over the support is the solid angle
        CHECK_NEAR(length(s->wi), 1.0, 1e-4);
        CHECK_NEAR(s->radiance.x, 4.0, 0);
    }
    CHECK_NEAR(estimate / n, expected, expected * 0.01);
}

TEST_CASE(light, sampled_pdf_agrees_with_light_pdf_for_hits) {
    Fixture f;
    f.scene.add_sphere({{0, 0, -6}, 1.5F, f.lamp});
    Triangle t;
    t.p0 = {-2, 3, -2};
    t.p1 = {2, 3, -2};
    t.p2 = {0, 3, -4};
    t.n0 = t.n1 = t.n2 = t.face_normal();
    t.material = f.lamp;
    f.scene.add_triangle(t);
    f.scene.build();
    Pcg32 rng(2);
    const Vec3 origins[] = {{0, 0, 0}, {0.5F, 1, -3}};
    for (const Vec3& from : origins) {
        int checked = 0;
        for (int i = 0; i < 2000; ++i) {
            const auto s = f.scene.sample_light(from, rng.next_real(), rng.next_vec2());
            if (!s) {
                continue;
            }
            Hit hit;
            if (!f.scene.intersect({from, s->wi}, kInf, hit) || !f.scene.material(hit.material).emissive()) {
                continue;
            }
            if (std::fabs(hit.t - s->distance) > 1e-3F * s->distance) {
                continue;  // another emitter was in front of the sampled point
            }
            CHECK_NEAR(f.scene.light_pdf(from, hit) / s->pdf, 1.0, 2e-3);
            ++checked;
        }
        CHECK(checked > 500);
    }
}

TEST_CASE(light, triangle_light_pdf_integrates_to_its_solid_angle) {
    Fixture f;
    Triangle t;
    t.p0 = {-1, 2, -1};
    t.p1 = {1.5F, 2.5F, -1};
    t.p2 = {0, 3, -3};
    t.n0 = t.n1 = t.n2 = t.face_normal();
    t.material = f.lamp;
    f.scene.add_triangle(t);
    f.scene.build();
    const Vec3 from(0.2F, 0, 0.5F);
    Pcg32 rng(3);
    double estimate = 0;
    const int n = 40000;
    for (int i = 0; i < n; ++i) {
        const auto s = f.scene.sample_light(from, rng.next_real(), rng.next_vec2());
        REQUIRE(s.has_value());
        estimate += 1.0 / s->pdf;
    }
    const double expected = triangle_solid_angle(from, t);
    CHECK_NEAR(estimate / n, expected, expected * 0.01);
}

TEST_CASE(light, sphere_light_seen_from_inside_uses_area_sampling) {
    Fixture f;
    f.scene.add_sphere({{0, 0, 0}, 4, f.lamp});
    f.scene.build();
    Pcg32 rng(4);
    double estimate = 0;
    const int n = 40000;
    for (int i = 0; i < n; ++i) {
        const auto s = f.scene.sample_light({0.5F, 0, 0}, rng.next_real(), rng.next_vec2());
        REQUIRE(s.has_value());
        estimate += 1.0 / s->pdf;
    }
    CHECK_NEAR(estimate / n, 4 * std::numbers::pi, 4 * std::numbers::pi * 0.01);  // the whole sphere of directions
}

TEST_CASE(light, selection_is_proportional_to_emitted_power) {
    Fixture f;
    Material dim;
    dim.base_texture = 0;
    dim.emission = Vec3(1);
    const auto dim_id = f.scene.add_material(dim);
    f.scene.add_sphere({{-5, 0, -5}, 1, f.lamp});   // power ~ 4 * area
    f.scene.add_sphere({{5, 0, -5}, 1, dim_id});    // power ~ 1 * area
    f.scene.build();
    Pcg32 rng(6);
    int left = 0;
    const int n = 40000;
    for (int i = 0; i < n; ++i) {
        const auto s = f.scene.sample_light({0, 0, 0}, rng.next_real(), rng.next_vec2());
        REQUIRE(s.has_value());
        left += s->wi.x < 0 ? 1 : 0;
    }
    CHECK_NEAR(static_cast<double>(left) / n, 0.8, 0.01);
}

TEST_CASE(light, no_lights_means_no_samples) {
    Fixture f;
    f.scene.add_sphere({{0, 0, -5}, 1, f.grey});
    f.scene.build();
    CHECK(!f.scene.sample_light({0, 0, 0}, 0.5F, {0.5F, 0.5F}).has_value());
}
