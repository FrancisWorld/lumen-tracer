#include "lumen/bsdf.hpp"
#include "lumen/rng.hpp"
#include "support.hpp"
#include "testing.hpp"

using namespace lumen;
using lumen::testing::integrate_sphere;

namespace {

Vec3 wo_at(Real theta) { return {std::sin(theta), 0, std::cos(theta)}; }

/// Directional albedo by quadrature: integral of f * cos over the hemisphere.
double albedo(const Bsdf& b, const Vec3& wo) {
    return integrate_sphere([&](const Vec3& wi) { return static_cast<double>(luminance(b.eval(wo, wi))) * std::max<double>(wi.z, 0.0); },
                            300, 300);
}

double mean_sample_weight(const Bsdf& b, const Vec3& wo, int n, std::uint64_t seed) {
    Pcg32 rng(seed);
    double sum = 0;
    for (int i = 0; i < n; ++i) {
        const Real u_lobe = rng.next_real();
        if (const auto s = b.sample(wo, u_lobe, rng.next_vec2())) {
            sum += luminance(s->weight);
        }
    }
    return sum / n;
}

}  // namespace

TEST_CASE(bsdf, fresnel_known_values) {
    CHECK_NEAR(fresnel_dielectric(1.0F, 1.0F / 1.5F), 0.04, 1e-5);  // glass at normal incidence
    CHECK_NEAR(fresnel_dielectric(1.0F, 1.5F), 0.04, 1e-5);         // symmetric from inside
    CHECK(fresnel_dielectric(0.01F, 1.0F / 1.5F) > 0.9);  // grazing reflects almost everything
    CHECK_NEAR(fresnel_dielectric(0.3F, 1.5F), 1.0, 0);             // beyond the critical angle: TIR
    CHECK_NEAR(fresnel_dielectric(0.5F, 1.0F), 0.0, 1e-6);          // index-matched interface is invisible
    CHECK_NEAR(fresnel_schlick(Vec3(0.04F), 1.0F).x, 0.04, 1e-6);
    CHECK_NEAR(fresnel_schlick(Vec3(0.04F), 0.0F).x, 1.0, 1e-6);
}

TEST_CASE(bsdf, lambertian_is_exactly_albedo) {
    const Bsdf b = Bsdf::standard({0.6F, 0.3F, 0.1F}, 0, 1, 0);
    const Vec3 wo = wo_at(0.7F);
    CHECK_NEAR(albedo(b, wo), luminance(Vec3(0.6F, 0.3F, 0.1F)), 0.003);
    Pcg32 rng(1);
    for (int i = 0; i < 100; ++i) {
        const auto s = b.sample(wo, rng.next_real(), rng.next_vec2());
        REQUIRE(s.has_value());
        CHECK_NEAR(s->weight.x, 0.6, 1e-4);  // cosine sampling cancels everything but the albedo
        CHECK_NEAR(s->weight.z, 0.1, 1e-4);
    }
}

TEST_CASE(bsdf, never_creates_energy_and_is_reciprocal_for_metals) {
    for (const Real rough : {0.05F, 0.3F, 0.7F, 1.0F}) {
        for (const Real metallic : {0.0F, 0.5F, 1.0F}) {
            const Bsdf b = Bsdf::standard({0.9F, 0.8F, 0.7F}, metallic, rough);
            for (const Real theta : {0.1F, 0.8F, 1.3F}) {
                CHECK(albedo(b, wo_at(theta)) <= 1.01);
            }
        }
    }
    const Bsdf metal = Bsdf::standard({0.9F, 0.6F, 0.3F}, 1, 0.4F);
    const Vec3 a = normalize(Vec3(0.3F, 0.2F, 0.8F));
    const Vec3 c = normalize(Vec3(-0.5F, 0.4F, 0.6F));
    CHECK_NEAR(metal.eval(a, c).x, metal.eval(c, a).x, 1e-4);
    CHECK_NEAR(metal.eval(a, c).z, metal.eval(c, a).z, 1e-4);
}

TEST_CASE(bsdf, sampling_is_consistent_with_eval_and_pdf) {
    // E[weight] over sampled directions must equal the quadrature albedo, which only holds when
    // the sampler, eval() and pdf() all describe the same distribution.
    for (const Real rough : {0.2F, 0.6F, 1.0F}) {
        for (const Real metallic : {0.0F, 1.0F}) {
            const Bsdf b = Bsdf::standard({0.8F, 0.8F, 0.8F}, metallic, rough);
            const Vec3 wo = wo_at(0.9F);
            const double expected = albedo(b, wo);
            CHECK_NEAR(mean_sample_weight(b, wo, 200000, 7), expected, 0.012);
        }
    }
}

TEST_CASE(bsdf, reported_pdf_matches_pdf_function_and_integrates_to_at_most_one) {
    const Bsdf b = Bsdf::standard({0.5F, 0.5F, 0.5F}, 0.3F, 0.4F);
    const Vec3 wo = wo_at(0.5F);
    Pcg32 rng(3);
    for (int i = 0; i < 2000; ++i) {
        if (const auto s = b.sample(wo, rng.next_real(), rng.next_vec2())) {
            CHECK_NEAR(s->pdf / b.pdf(wo, s->wi), 1.0, 1e-4);
            CHECK(!s->specular);
            CHECK(s->wi.z > 0);
        }
    }
    const double total = integrate_sphere([&](const Vec3& wi) { return static_cast<double>(b.pdf(wo, wi)); }, 400, 400);
    CHECK(total <= 1.0 + 0.01);
    CHECK(total > 0.9);  // only directions reflected below the horizon are lost
    CHECK_NEAR(b.pdf(Vec3(0, 0, -1), Vec3(0, 0, 1)), 0.0, 0);
    CHECK_NEAR(b.pdf(wo, Vec3(0, 0, -1)), 0.0, 0);
}

TEST_CASE(bsdf, lower_hemisphere_and_degenerate_inputs) {
    const Bsdf b = Bsdf::standard({1, 1, 1}, 0, 0.5F);
    CHECK(!b.sample(Vec3(0, 0, -1), 0.5F, {0.5F, 0.5F}).has_value());
    CHECK_NEAR(max_component(b.eval(wo_at(0.3F), Vec3(0, 0, -1))), 0.0, 0);
    // roughness 0 is clamped, so the lobe stays finite
    const Bsdf mirror = Bsdf::standard({1, 1, 1}, 1, 0);
    const auto s = mirror.sample(Vec3(0, 0, 1), 0.5F, {0.3F, 0.3F});
    REQUIRE(s.has_value());
    CHECK(is_finite(s->weight));
}

TEST_CASE(bsdf, dielectric_obeys_snell_and_conserves_energy) {
    const Real ior = 1.5F;
    const Bsdf glass = Bsdf::dielectric(ior, Vec3(1), true);
    CHECK(glass.is_specular());
    CHECK_NEAR(max_component(glass.eval(Vec3(0, 0, 1), Vec3(0, 0, 1))), 0.0, 0);
    CHECK_NEAR(glass.pdf(Vec3(0, 0, 1), Vec3(0, 0, 1)), 0.0, 0);

    const Vec3 wo = wo_at(0.8F);
    const auto refracted = glass.sample(wo, 0.999F, {});
    REQUIRE(refracted.has_value());
    CHECK(refracted->transmission);
    CHECK(refracted->wi.z < 0);
    CHECK_NEAR(length(refracted->wi), 1.0, 1e-5);
    const Real sin_i = std::sqrt(1 - wo.z * wo.z);
    const Real sin_t = std::sqrt(1 - refracted->wi.z * refracted->wi.z);
    CHECK_NEAR(sin_i, ior * sin_t, 1e-5);  // Snell: sin_i = n * sin_t

    const auto reflected = glass.sample(wo, 0.0F, {});
    REQUIRE(reflected.has_value());
    CHECK(!reflected->transmission);
    CHECK_NEAR(reflected->wi.x, -wo.x, 1e-6);
    CHECK_NEAR(reflected->wi.z, wo.z, 1e-6);

    // Reflect/refract split follows Fresnel and every sample carries weight 1 (lossless, white tint).
    Pcg32 rng(5);
    int reflected_count = 0;
    const int n = 100000;
    for (int i = 0; i < n; ++i) {
        const auto s = glass.sample(wo, rng.next_real(), {});
        REQUIRE(s.has_value());
        CHECK_NEAR(s->weight.y, 1.0, 1e-6);
        reflected_count += s->transmission ? 0 : 1;
    }
    CHECK_NEAR(static_cast<double>(reflected_count) / n, fresnel_dielectric(wo.z, 1.0F / ior), 0.005);
}

TEST_CASE(bsdf, dielectric_total_internal_reflection_and_tint) {
    const Bsdf inside = Bsdf::dielectric(1.5F, Vec3(0.5F, 1, 1), false);
    const Vec3 grazing = wo_at(1.4F);  // beyond the critical angle of ~41.8 degrees
    for (const Real u : {0.0F, 0.5F, 0.999F}) {
        const auto s = inside.sample(grazing, u, {});
        REQUIRE(s.has_value());
        CHECK(!s->transmission);
        CHECK(s->wi.z > 0);
    }
    const auto out = inside.sample(Vec3(0, 0, 1), 0.999F, {});
    REQUIRE(out.has_value());
    CHECK(out->transmission);
    CHECK_NEAR(out->weight.x, 0.5, 1e-6);  // tint applies to transmission only
    CHECK_NEAR(out->wi.z, -1.0, 1e-5);
}
