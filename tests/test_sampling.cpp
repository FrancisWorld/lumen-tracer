#include "lumen/microfacet.hpp"
#include "lumen/rng.hpp"
#include "lumen/sampling.hpp"
#include "support.hpp"
#include "testing.hpp"

using namespace lumen;
using lumen::testing::integrate_sphere;


TEST_CASE(sampling, concentric_disk_stays_in_unit_disk_and_is_area_uniform) {
    Pcg32 rng(1);
    int inside_half = 0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        const Vec2 p = sample_disk_concentric(rng.next_vec2());
        const Real r2 = p.x * p.x + p.y * p.y;
        CHECK(r2 <= 1.0001F);
        inside_half += r2 <= 0.25F ? 1 : 0;  // radius 0.5 covers a quarter of the area
    }
    CHECK_NEAR(static_cast<double>(inside_half) / n, 0.25, 0.01);
}

TEST_CASE(sampling, cosine_pdf_integrates_to_one_and_matches_samples) {
    const double total =
        integrate_sphere([](const Vec3& w) { return static_cast<double>(cosine_hemisphere_pdf(w.z)); });
    CHECK_NEAR(total, 1.0, 2e-3);

    // Sampler agrees with the pdf: E[cos] under pdf cos/pi is 2/3.
    Pcg32 rng(3);
    double mean_cos = 0;
    const int n = 200000;
    for (int i = 0; i < n; ++i) {
        const Vec3 w = sample_cosine_hemisphere(rng.next_vec2());
        CHECK_NEAR(length(w), 1.0, 1e-4);
        mean_cos += w.z;
    }
    CHECK_NEAR(mean_cos / n, 2.0 / 3.0, 0.004);
}

TEST_CASE(sampling, uniform_sphere_pdf_integrates_to_one) {
    const double total = integrate_sphere([](const Vec3&) { return static_cast<double>(uniform_sphere_pdf()); }, 100, 200);
    CHECK_NEAR(total, 1.0, 1e-3);
    Pcg32 rng(9);
    Vec3 mean;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        const Vec3 w = sample_uniform_sphere(rng.next_vec2());
        CHECK_NEAR(length(w), 1.0, 1e-4);
        mean += w;
    }
    CHECK(length(mean / static_cast<Real>(n)) < 0.01F);
}

TEST_CASE(sampling, cone_pdf_integrates_to_one_over_its_support) {
    for (const Real cos_max : {0.99F, 0.8F, 0.0F, -0.5F}) {
        const double total = integrate_sphere(
            [cos_max](const Vec3& w) { return w.z >= cos_max ? static_cast<double>(uniform_cone_pdf(cos_max)) : 0.0; },
            600, 300);
        CHECK_NEAR(total, 1.0, 0.01);
    }
    Pcg32 rng(11);
    for (int i = 0; i < 5000; ++i) {
        const Vec3 w = sample_uniform_cone(rng.next_vec2(), 0.9F);
        CHECK(w.z >= 0.9F - 1e-5F);
        CHECK_NEAR(length(w), 1.0, 1e-4);
    }
}

TEST_CASE(sampling, triangle_sampling_is_uniform_in_area) {
    Pcg32 rng(5);
    // Centroid of uniformly distributed barycentrics is (1/3, 1/3, 1/3).
    double b0 = 0;
    double b1 = 0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        const Vec3 b = sample_uniform_triangle(rng.next_vec2());
        CHECK_NEAR(b.x + b.y + b.z, 1.0, 1e-5);
        CHECK(b.x >= 0 && b.y >= 0 && b.z >= -1e-6F);
        b0 += b.x;
        b1 += b.y;
    }
    CHECK_NEAR(b0 / n, 1.0 / 3.0, 0.004);
    CHECK_NEAR(b1 / n, 1.0 / 3.0, 0.004);
}

TEST_CASE(sampling, power_heuristic_weights_sum_to_one) {
    CHECK_NEAR(power_heuristic(0.3F, 0.7F) + power_heuristic(0.7F, 0.3F), 1.0, 1e-6);
    CHECK_NEAR(power_heuristic(0, 0), 0.0, 0.0);
    CHECK_NEAR(power_heuristic(1, 0), 1.0, 1e-6);
}

TEST_CASE(ggx, ndf_is_normalised_in_projected_area) {
    // Definition of a microfacet NDF: integral of D(h) cos(theta_h) over the hemisphere is 1.
    for (const Real alpha : {0.1F, 0.3F, 0.6F, 1.0F}) {
        const double total = integrate_sphere(
            [alpha](const Vec3& h) { return static_cast<double>(ggx::D(alpha, h.z)) * std::max<double>(h.z, 0.0); },
            1500, 64);
        CHECK_NEAR(total, 1.0, 0.01);
    }
}

TEST_CASE(ggx, visible_normal_pdf_integrates_to_one) {
    for (const Real alpha : {0.15F, 0.5F, 0.9F}) {
        for (const Real theta : {0.0F, 0.6F, 1.2F, 1.5F}) {
            const Vec3 wo(std::sin(theta), 0, std::cos(theta));
            const double total = integrate_sphere(
                [&](const Vec3& h) { return static_cast<double>(ggx::visible_normal_pdf(alpha, wo, h)); }, 1500, 96);
            CHECK_NEAR(total, 1.0, 0.02);
        }
    }
}

TEST_CASE(ggx, sampled_normals_follow_visible_normal_distribution) {
    // Compare E[h.z] from the sampler against quadrature of h.z * pdf(h).
    const Real alpha = 0.4F;
    const Vec3 wo = normalize(Vec3(0.7F, 0.1F, 0.5F));
    const double expected = integrate_sphere(
        [&](const Vec3& h) { return h.z * static_cast<double>(ggx::visible_normal_pdf(alpha, wo, h)); }, 1500, 96);
    Pcg32 rng(21);
    double mean = 0;
    const int n = 300000;
    for (int i = 0; i < n; ++i) {
        const Vec3 h = ggx::sample_visible_normal(alpha, wo, rng.next_vec2());
        CHECK(h.z > 0);
        mean += h.z;
    }
    CHECK_NEAR(mean / n, expected, 0.005);
}

TEST_CASE(ggx, smith_masking_limits) {
    CHECK_NEAR(ggx::G1(0.5F, 1.0F), 1.0, 1e-6);          // normal incidence is never masked
    CHECK(ggx::G1(0.5F, 0.1F) < ggx::G1(0.5F, 0.9F));      // grazing angles are masked more
    CHECK(ggx::G2(0.5F, 0.4F, 0.4F) <= ggx::G1(0.5F, 0.4F));  // joint masking cannot exceed single
    CHECK_NEAR(ggx::G1(1e-4F, 0.05F), 1.0, 1e-3);          // a nearly flat surface never masks
}
