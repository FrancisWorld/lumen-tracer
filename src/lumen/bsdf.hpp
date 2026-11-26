// Surface scattering models, evaluated in the local shading frame (+z is the
// normal, always on the side the ray arrived from).
//
//  * Standard: GGX specular lobe (Smith height-correlated shadowing, Schlick
//    Fresnel, visible-normal sampling) over a Lambertian diffuse base,
//    blended by `metallic`. Two lobes are sampled stochastically and the pdf
//    is the exact mixture, so multiple importance sampling stays unbiased.
//  * Dielectric: perfectly smooth glass with exact unpolarised Fresnel.
#pragma once

#include <optional>

#include "lumen/math.hpp"

namespace lumen {

struct BsdfSample {
    Vec3 wi;        // local frame
    Vec3 weight;    // f * |cos| / pdf
    Real pdf = 0;   // solid angle; meaningless for specular samples
    bool specular = false;
    bool transmission = false;
};

/// Exact Fresnel reflectance for unpolarised light. eta = n_incident / n_transmitted.
Real fresnel_dielectric(Real cos_i, Real eta);
Vec3 fresnel_schlick(const Vec3& f0, Real cos_theta);

class Bsdf {
public:
    /// `specular` in [0,1] scales the dielectric reflectance at normal incidence (0.5 = 4%); 0 gives pure Lambertian.
    static Bsdf standard(const Vec3& base_color, Real metallic, Real roughness, Real specular = Real(0.5));
    static Bsdf dielectric(Real ior, const Vec3& tint, bool front_face);

    /// True for delta distributions: eval/pdf are zero and light sampling must be skipped.
    bool is_specular() const { return kind_ == Kind::Dielectric; }

    /// BSDF value (without the cosine term).
    Vec3 eval(const Vec3& wo, const Vec3& wi) const;
    /// Solid-angle density that sample() would produce `wi`.
    Real pdf(const Vec3& wo, const Vec3& wi) const;
    /// `u_lobe` picks the lobe / reflect-vs-refract; `u` drives the direction.
    std::optional<BsdfSample> sample(const Vec3& wo, Real u_lobe, Vec2 u) const;

private:
    enum class Kind { Standard, Dielectric };
    Real specular_probability() const;

    Kind kind_ = Kind::Standard;
    Vec3 base_{};
    Vec3 f0_{};
    Real metallic_ = 0;
    Real alpha_ = 1;
    Real coat_f0_ = 0.04F;
    bool has_specular_ = true;  // false: pure Lambertian (no grazing-angle Fresnel either)
    Real ior_ = 1.5F;
    Vec3 tint_{1, 1, 1};
    bool front_face_ = true;
};

}  // namespace lumen
