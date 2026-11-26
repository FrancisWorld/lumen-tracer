// Unidirectional path tracer with next-event estimation, multiple importance
// sampling (power heuristic) and Russian roulette.
#pragma once

#include "lumen/rng.hpp"
#include "lumen/scene.hpp"

namespace lumen {

struct PathSettings {
    int max_depth = 8;        // maximum number of scattering events
    int roulette_depth = 3;   // Russian roulette starts after this many bounces
    Real sample_clamp = 0;    // 0 disables; otherwise caps the largest channel of each sample (biased, tames fireflies)
};

/// Auxiliary data from the first non-specular surface, used to guide the denoiser.
struct PathGuides {
    Vec3 albedo{1, 1, 1};
    Vec3 normal{};
    Real depth = kInf;
};

struct PathSample {
    Vec3 radiance;
    PathGuides guides;
};

/// Radiance along `ray`, estimated with one path.
PathSample trace_path(const Scene& scene, Ray ray, Pcg32& rng, const PathSettings& settings);

/// Origin for a secondary ray leaving `hit` along `direction`, nudged off the surface to avoid self-intersection.
Vec3 offset_ray_origin(const Hit& hit, const Vec3& direction);

}  // namespace lumen
