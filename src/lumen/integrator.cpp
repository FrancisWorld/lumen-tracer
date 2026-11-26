#include "lumen/integrator.hpp"

#include "lumen/sampling.hpp"

namespace lumen {

Vec3 offset_ray_origin(const Hit& hit, const Vec3& direction) {
    const Real scale = Real(1) + max_component(vabs(hit.p));
    const Real eps = Real(1e-4) * scale;
    return hit.p + hit.n_geo * (dot(direction, hit.n_geo) > 0 ? eps : -eps);
}

PathSample trace_path(const Scene& scene, Ray ray, Pcg32& rng, const PathSettings& settings) {
    PathSample result;
    Vec3 radiance;
    Vec3 beta(1);
    bool previous_specular = true;  // camera rays count as specular: emission is taken at full weight
    Real previous_pdf = 0;
    Vec3 previous_origin = ray.origin;
    bool guides_set = false;
    Real path_length = 0;

    for (int bounce = 0;; ++bounce) {
        Hit hit;
        if (!scene.intersect(ray, kInf, hit)) {
            radiance += beta * scene.environment(ray.direction);
            break;
        }
        path_length += hit.t;
        const Material& material = scene.material(hit.material);

        if (material.emissive()) {
            Real weight = 1;
            if (!previous_specular) {
                weight = power_heuristic(previous_pdf, scene.light_pdf(previous_origin, hit));
            }
            radiance += beta * material.emission * weight;
        }

        const Bsdf bsdf = scene.bsdf_at(hit);
        if (!guides_set && !bsdf.is_specular()) {
            guides_set = true;
            result.guides.albedo = material.emissive() ? Vec3(1) : scene.base_color(material, hit);
            result.guides.normal = hit.n_shade;
            result.guides.depth = path_length;
        }
        if (bounce >= settings.max_depth) {
            break;
        }

        const Onb frame = Onb::from_normal(hit.n_shade);
        const Vec3 wo = frame.to_local(-ray.direction);

        // Next-event estimation: sample a light and weight against BSDF sampling.
        if (!bsdf.is_specular() && scene.light_count() > 0) {
            const Real u_select = rng.next_real();
            const Vec2 u_light = rng.next_vec2();
            const Vec3 origin = offset_ray_origin(hit, hit.n_geo);  // shading happens on the ray's side
            if (const auto light = scene.sample_light(origin, u_select, u_light)) {
                const Vec3 wi_local = frame.to_local(light->wi);
                if (wi_local.z > 0 && dot(light->wi, hit.n_geo) > 0 && light->pdf > 0) {
                    const Vec3 f = bsdf.eval(wo, wi_local);
                    if (max_component(f) > 0) {
                        const Real bsdf_pdf = bsdf.pdf(wo, wi_local);
                        const Real weight = power_heuristic(light->pdf, bsdf_pdf);
                        const Ray shadow{origin, light->wi};
                        if (!scene.occluded(shadow, light->distance * (Real(1) - Real(2e-3)))) {
                            radiance += beta * f * light->radiance * (wi_local.z * weight / light->pdf);
                        }
                    }
                }
            }
        }

        const Real u_lobe = rng.next_real();
        const Vec2 u_dir = rng.next_vec2();
        const auto sample = bsdf.sample(wo, u_lobe, u_dir);
        if (!sample || !is_finite(sample->weight)) {
            break;
        }
        const Vec3 wi = normalize(frame.to_world(sample->wi));
        if (!sample->transmission && dot(wi, hit.n_geo) <= 0) {
            break;  // shading normal pushed the direction below the true surface
        }
        beta *= sample->weight;
        previous_specular = sample->specular;
        previous_pdf = sample->pdf;
        previous_origin = offset_ray_origin(hit, wi);
        ray = {previous_origin, wi};

        if (bounce >= settings.roulette_depth) {
            const Real survive = std::clamp(max_component(beta), Real(0.05), Real(0.95));
            if (rng.next_real() >= survive) {
                break;
            }
            beta = beta / survive;
        }
    }

    if (!is_finite(radiance)) {
        radiance = Vec3(0);
    }
    if (settings.sample_clamp > 0) {
        const Real peak = max_component(radiance);
        if (peak > settings.sample_clamp) {
            radiance = radiance * (settings.sample_clamp / peak);
        }
    }
    result.radiance = radiance;
    return result;
}

}  // namespace lumen
