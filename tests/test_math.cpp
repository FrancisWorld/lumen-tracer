#include "lumen/math.hpp"
#include "lumen/rng.hpp"
#include "testing.hpp"

using namespace lumen;

TEST_CASE(math, vector_algebra) {
    const Vec3 a(1, 2, 3);
    const Vec3 b(-2, 0.5F, 4);
    CHECK_NEAR(dot(a, b), 11.0, 1e-6);
    const Vec3 c = cross(a, b);
    CHECK_NEAR(dot(c, a), 0.0, 1e-5);
    CHECK_NEAR(dot(c, b), 0.0, 1e-5);
    CHECK_NEAR(length(normalize(a)), 1.0, 1e-6);
    CHECK_NEAR(luminance(Vec3(1)), 1.0, 1e-6);
    const Vec3 r = reflect(Vec3(1, -1, 0), Vec3(0, 1, 0));
    CHECK_NEAR(r.x, 1.0, 1e-6);
    CHECK_NEAR(r.y, 1.0, 1e-6);
}

TEST_CASE(math, onb_is_orthonormal_for_all_hemispheres) {
    Pcg32 rng(7);
    for (int i = 0; i < 2000; ++i) {
        const Vec3 n = normalize(Vec3(rng.next_real() - 0.5F, rng.next_real() - 0.5F, rng.next_real() - 0.5F));
        const Onb f = Onb::from_normal(n);
        CHECK_NEAR(length(f.t), 1.0, 1e-4);
        CHECK_NEAR(length(f.b), 1.0, 1e-4);
        CHECK_NEAR(dot(f.t, f.b), 0.0, 1e-4);
        CHECK_NEAR(dot(f.t, f.n), 0.0, 1e-4);
        CHECK_NEAR(dot(f.b, f.n), 0.0, 1e-4);
        const Vec3 v(0.3F, -0.4F, 0.8F);
        const Vec3 round_trip = f.to_world(f.to_local(v));
        CHECK_NEAR(length(round_trip - v), 0.0, 1e-4);
    }
    // The poles are where naive constructions break.
    const Onb up = Onb::from_normal({0, 0, 1});
    const Onb down = Onb::from_normal({0, 0, -1});
    CHECK_NEAR(dot(up.t, up.b), 0.0, 1e-6);
    CHECK_NEAR(dot(down.t, down.b), 0.0, 1e-6);
    CHECK(is_finite(down.t) && is_finite(down.b));
}

TEST_CASE(math, matrix_inverse_round_trips) {
    const Mat3 m{{2, 0.5F, 0}, {-1, 3, 0.25F}, {0.1F, 0, 1.5F}};
    const Mat3 id = m * inverse(m);
    CHECK_NEAR(id.r0.x, 1.0, 1e-5);
    CHECK_NEAR(id.r1.y, 1.0, 1e-5);
    CHECK_NEAR(id.r2.z, 1.0, 1e-5);
    CHECK_NEAR(id.r0.y, 0.0, 1e-5);
    CHECK_NEAR(id.r2.x, 0.0, 1e-5);
}

TEST_CASE(math, transform_rotates_scales_translates) {
    const Transform t = Transform::from_trs({1, 2, 3}, {0, 90, 0}, {2, 2, 2});
    const Vec3 p = t.point({1, 0, 0});  // scale -> (2,0,0), rotate Y 90 -> (0,0,-2), translate
    CHECK_NEAR(p.x, 1.0, 1e-5);
    CHECK_NEAR(p.y, 2.0, 1e-5);
    CHECK_NEAR(p.z, 1.0, 1e-5);
    CHECK(!t.flips_winding());
    CHECK(Transform::from_trs({}, {}, {-1, 1, 1}).flips_winding());
}

TEST_CASE(math, normals_stay_perpendicular_under_non_uniform_scale) {
    const Transform t = Transform::from_trs({}, {20, 35, 10}, {3, 1, 0.5F});
    const Vec3 tangent = normalize(Vec3(1, -1, 0));
    const Vec3 n = normalize(Vec3(1, 1, 0));  // perpendicular to tangent
    CHECK_NEAR(dot(t.vector(tangent), t.normal(n)), 0.0, 1e-5);
}

TEST_CASE(rng, uniform_in_unit_interval_with_correct_moments) {
    Pcg32 rng(123, 5);
    double sum = 0;
    double sum2 = 0;
    const int n = 200000;
    for (int i = 0; i < n; ++i) {
        const double v = rng.next_real();
        CHECK(v >= 0.0 && v < 1.0);
        sum += v;
        sum2 += v * v;
    }
    CHECK_NEAR(sum / n, 0.5, 0.003);
    CHECK_NEAR(sum2 / n - (sum / n) * (sum / n), 1.0 / 12.0, 0.002);
}

TEST_CASE(rng, pixel_streams_are_reproducible_and_distinct) {
    Pcg32 a = pixel_rng(42, 10, 20);
    Pcg32 b = pixel_rng(42, 10, 20);
    Pcg32 c = pixel_rng(42, 11, 20);
    Pcg32 d = pixel_rng(43, 10, 20);
    bool same_c = true;
    bool same_d = true;
    for (int i = 0; i < 16; ++i) {
        const auto va = a.next_u32();
        CHECK_EQ(va, b.next_u32());
        same_c = same_c && va == c.next_u32();
        same_d = same_d && va == d.next_u32();
    }
    CHECK(!same_c);
    CHECK(!same_d);
}
