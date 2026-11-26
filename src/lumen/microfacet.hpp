// Isotropic GGX (Trowbridge-Reitz) microfacet model, all in the local shading
// frame where the surface normal is +z.
#pragma once

#include "lumen/math.hpp"

namespace lumen::ggx {

/// Normal distribution function D(h).
inline Real D(Real alpha, Real cos_h) {
    if (cos_h <= 0) {
        return 0;
    }
    const Real a2 = alpha * alpha;
    const Real d = (a2 - Real(1)) * cos_h * cos_h + Real(1);
    return a2 / (kPi * d * d);
}

/// Smith auxiliary function Lambda(w).
inline Real lambda(Real alpha, Real cos_w) {
    const Real c2 = cos_w * cos_w;
    if (c2 <= 0) {
        return 0;
    }
    const Real tan2 = (Real(1) - c2) / c2;
    return (std::sqrt(Real(1) + alpha * alpha * tan2) - Real(1)) / Real(2);
}

inline Real G1(Real alpha, Real cos_w) { return Real(1) / (Real(1) + lambda(alpha, cos_w)); }

/// Height-correlated masking-shadowing G2(wo, wi).
inline Real G2(Real alpha, Real cos_o, Real cos_i) {
    return Real(1) / (Real(1) + lambda(alpha, cos_o) + lambda(alpha, cos_i));
}

/// Density of visible normals D_wo(h) with respect to solid angle around h.
inline Real visible_normal_pdf(Real alpha, const Vec3& wo, const Vec3& h) {
    if (wo.z <= 0) {
        return 0;
    }
    return G1(alpha, wo.z) * std::max(Real(0), dot(wo, h)) * D(alpha, h.z) / wo.z;
}

/// Sample a visible microfacet normal (Heitz 2018, "Sampling the GGX Distribution of Visible Normals").
inline Vec3 sample_visible_normal(Real alpha, const Vec3& wo, Vec2 u) {
    const Vec3 vh = normalize(Vec3(alpha * wo.x, alpha * wo.y, wo.z));
    const Real lensq = vh.x * vh.x + vh.y * vh.y;
    const Vec3 t1 = lensq > 0 ? Vec3(-vh.y, vh.x, 0) / std::sqrt(lensq) : Vec3(1, 0, 0);
    const Vec3 t2 = cross(vh, t1);
    const Real r = std::sqrt(u.x);
    const Real phi = Real(2) * kPi * u.y;
    const Real p1 = r * std::cos(phi);
    Real p2 = r * std::sin(phi);
    const Real s = Real(0.5) * (Real(1) + vh.z);
    p2 = (Real(1) - s) * safe_sqrt(Real(1) - p1 * p1) + s * p2;
    const Vec3 nh = t1 * p1 + t2 * p2 + vh * safe_sqrt(Real(1) - p1 * p1 - p2 * p2);
    return normalize(Vec3(alpha * nh.x, alpha * nh.y, std::max(Real(1e-6), nh.z)));
}

}  // namespace lumen::ggx
