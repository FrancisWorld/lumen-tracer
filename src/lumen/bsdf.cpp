#include "lumen/bsdf.hpp"

#include "lumen/microfacet.hpp"
#include "lumen/sampling.hpp"

namespace lumen {
namespace {
constexpr Real kMinAlpha = 0.005F;
}  // namespace

Real fresnel_dielectric(Real cos_i, Real eta) {
    cos_i = clamp01(cos_i);
    const Real sin2_t = eta * eta * (Real(1) - cos_i * cos_i);
    if (sin2_t >= 1) {
        return 1;  // total internal reflection
    }
    const Real cos_t = std::sqrt(Real(1) - sin2_t);
    const Real rs = (eta * cos_i - cos_t) / (eta * cos_i + cos_t);
    const Real rp = (cos_i - eta * cos_t) / (cos_i + eta * cos_t);
    return Real(0.5) * (rs * rs + rp * rp);
}

Vec3 fresnel_schlick(const Vec3& f0, Real cos_theta) {
    const Real m = Real(1) - clamp01(cos_theta);
    const Real m2 = m * m;
    return f0 + (Vec3(1) - f0) * (m2 * m2 * m);
}

Bsdf Bsdf::standard(const Vec3& base_color, Real metallic, Real roughness, Real specular) {
    Bsdf b;
    b.kind_ = Kind::Standard;
    b.base_ = base_color;
    b.metallic_ = clamp01(metallic);
    b.coat_f0_ = Real(0.08) * clamp01(specular);
    b.has_specular_ = b.metallic_ > 0 || specular > 0;
    b.f0_ = lerp(Vec3(b.coat_f0_), base_color, b.metallic_);
    b.alpha_ = std::max(kMinAlpha, clamp01(roughness) * clamp01(roughness));
    return b;
}

Bsdf Bsdf::dielectric(Real ior, const Vec3& tint, bool front_face) {
    Bsdf b;
    b.kind_ = Kind::Dielectric;
    b.ior_ = ior;
    b.tint_ = tint;
    b.front_face_ = front_face;
    return b;
}

Real Bsdf::specular_probability() const {
    if (!has_specular_) {
        return 0;
    }
    const Real ws = luminance(f0_);
    const Real wd = (Real(1) - metallic_) * luminance(base_);
    if (wd <= 0) {
        return 1;
    }
    return std::clamp(ws / (ws + wd), Real(0.1), Real(0.9));
}

Vec3 Bsdf::eval(const Vec3& wo, const Vec3& wi) const {
    if (kind_ != Kind::Standard || wo.z <= 0 || wi.z <= 0) {
        return Vec3(0);
    }
    const Vec3 h = normalize(wo + wi);
    const Real wo_h = dot(wo, h);
    const Real coat = has_specular_ ? Real(1) - fresnel_schlick(Vec3(coat_f0_), wo.z).x : Real(1);
    const Vec3 diffuse = base_ * ((Real(1) - metallic_) * coat * kInvPi);
    if (!has_specular_) {
        return diffuse;
    }
    const Vec3 fresnel = fresnel_schlick(f0_, wo_h);
    const Real spec_scale = ggx::D(alpha_, h.z) * ggx::G2(alpha_, wo.z, wi.z) / (Real(4) * wo.z * wi.z);
    return diffuse + fresnel * spec_scale;
}

Real Bsdf::pdf(const Vec3& wo, const Vec3& wi) const {
    if (kind_ != Kind::Standard || wo.z <= 0 || wi.z <= 0) {
        return 0;
    }
    const Vec3 h = normalize(wo + wi);
    const Real wo_h = dot(wo, h);
    if (wo_h <= 0) {
        return 0;
    }
    const Real p_spec = specular_probability();
    const Real pdf_spec = ggx::visible_normal_pdf(alpha_, wo, h) / (Real(4) * wo_h);
    return p_spec * pdf_spec + (Real(1) - p_spec) * cosine_hemisphere_pdf(wi.z);
}

std::optional<BsdfSample> Bsdf::sample(const Vec3& wo, Real u_lobe, Vec2 u) const {
    if (wo.z <= 0) {
        return std::nullopt;
    }
    if (kind_ == Kind::Dielectric) {
        const Real eta = front_face_ ? Real(1) / ior_ : ior_;
        const Real f = fresnel_dielectric(wo.z, eta);
        if (u_lobe < f) {
            return BsdfSample{Vec3(-wo.x, -wo.y, wo.z), Vec3(1), 1, true, false};
        }
        const Real cos_t = safe_sqrt(Real(1) - eta * eta * (Real(1) - wo.z * wo.z));
        return BsdfSample{Vec3(-eta * wo.x, -eta * wo.y, -cos_t), tint_, 1, true, true};
    }

    const Real p_spec = specular_probability();
    Vec3 wi;
    if (u_lobe < p_spec) {
        const Vec3 h = ggx::sample_visible_normal(alpha_, wo, u);
        wi = reflect(-wo, h);
    } else {
        wi = sample_cosine_hemisphere(u);
    }
    if (wi.z <= 0) {
        return std::nullopt;
    }
    const Real pdf_value = pdf(wo, wi);
    if (pdf_value <= 0) {
        return std::nullopt;
    }
    return BsdfSample{wi, eval(wo, wi) * (wi.z / pdf_value), pdf_value, false, false};
}

}  // namespace lumen
