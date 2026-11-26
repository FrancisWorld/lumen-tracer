// Sampling routines and their PDFs. Every sampler here has a matching pdf
// function and the tests integrate each pdf to check it against the sampler.
#pragma once

#include "lumen/math.hpp"

namespace lumen {

/// Shirley-Chiu concentric mapping of the unit square to the unit disk.
inline Vec2 sample_disk_concentric(Vec2 u) {
    const Real ox = Real(2) * u.x - Real(1);
    const Real oy = Real(2) * u.y - Real(1);
    if (ox == 0 && oy == 0) {
        return {0, 0};
    }
    Real r = 0;
    Real theta = 0;
    if (std::fabs(ox) > std::fabs(oy)) {
        r = ox;
        theta = (kPi / 4) * (oy / ox);
    } else {
        r = oy;
        theta = kPi / 2 - (kPi / 4) * (ox / oy);
    }
    return {r * std::cos(theta), r * std::sin(theta)};
}

/// Cosine-weighted direction on the +z hemisphere.
inline Vec3 sample_cosine_hemisphere(Vec2 u) {
    const Vec2 d = sample_disk_concentric(u);
    return {d.x, d.y, safe_sqrt(Real(1) - d.x * d.x - d.y * d.y)};
}
constexpr Real cosine_hemisphere_pdf(Real cos_theta) { return cos_theta > 0 ? cos_theta * kInvPi : Real(0); }

inline Vec3 sample_uniform_sphere(Vec2 u) {
    const Real z = Real(1) - Real(2) * u.x;
    const Real r = safe_sqrt(Real(1) - z * z);
    const Real phi = Real(2) * kPi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), z};
}
constexpr Real uniform_sphere_pdf() { return Real(1) / (Real(4) * kPi); }

/// Uniform direction inside the cone around +z with the given cosine half-angle.
inline Vec3 sample_uniform_cone(Vec2 u, Real cos_theta_max) {
    const Real cos_theta = (Real(1) - u.x) + u.x * cos_theta_max;
    const Real sin_theta = safe_sqrt(Real(1) - cos_theta * cos_theta);
    const Real phi = Real(2) * kPi * u.y;
    return {sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
}
constexpr Real uniform_cone_pdf(Real cos_theta_max) { return Real(1) / (Real(2) * kPi * (Real(1) - cos_theta_max)); }

/// Uniform barycentric coordinates (b0, b1, b2) on a triangle.
inline Vec3 sample_uniform_triangle(Vec2 u) {
    const Real su = std::sqrt(u.x);
    const Real b0 = Real(1) - su;
    const Real b1 = u.y * su;
    return {b0, b1, Real(1) - b0 - b1};
}

/// Balance of two pdfs for multiple importance sampling (power heuristic, beta = 2).
constexpr Real power_heuristic(Real pdf_a, Real pdf_b) {
    const Real a = pdf_a * pdf_a;
    const Real b = pdf_b * pdf_b;
    return (a + b) > 0 ? a / (a + b) : Real(0);
}

}  // namespace lumen
