// Thin-lens perspective camera. aperture == 0 degenerates to a pinhole.
#pragma once

#include "lumen/geometry.hpp"

namespace lumen {

struct CameraParams {
    Vec3 look_from{0, 0, 5};
    Vec3 look_at{0, 0, 0};
    Vec3 up{0, 1, 0};
    Real vertical_fov_degrees = 40;
    Real aperture = 0;        // lens diameter in world units
    Real focus_distance = 0;  // 0 = distance to look_at
};

class Camera {
public:
    Camera() : Camera(CameraParams{}, 1) {}
    /// Throws InputError for degenerate parameters (zero view direction, up parallel to view, fov out of range).
    Camera(const CameraParams& params, Real aspect_ratio);

    /// `film` is the position in pixels from the top-left corner (continuous), `lens` a uniform sample in [0,1)^2.
    Ray generate_ray(Vec2 film, Vec2 image_size, Vec2 lens) const;

    const Vec3& forward() const { return forward_; }
    Real focus_distance() const { return focus_; }

private:
    Vec3 origin_;
    Vec3 u_, v_, forward_;
    Vec3 lower_left_, horizontal_, vertical_;
    Real lens_radius_ = 0;
    Real focus_ = 1;
};

}  // namespace lumen
