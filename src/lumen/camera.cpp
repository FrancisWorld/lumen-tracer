#include "lumen/camera.hpp"

#include "lumen/error.hpp"
#include "lumen/sampling.hpp"

namespace lumen {

Camera::Camera(const CameraParams& p, Real aspect) {
    const Vec3 view = p.look_at - p.look_from;
    if (length_squared(view) < Real(1e-12)) {
        throw InputError("camera: look_from and look_at coincide");
    }
    if (!(p.vertical_fov_degrees > 0 && p.vertical_fov_degrees < 179) || !(aspect > 0) || p.aperture < 0) {
        throw InputError("camera: fov must be in (0, 179) degrees, aspect positive and aperture non-negative");
    }
    forward_ = normalize(view);
    const Vec3 right = cross(forward_, p.up);
    if (length_squared(right) < Real(1e-12)) {
        throw InputError("camera: up vector is parallel to the view direction");
    }
    u_ = normalize(right);
    v_ = cross(u_, forward_);
    focus_ = p.focus_distance > 0 ? p.focus_distance : length(view);
    const Real half_h = std::tan(radians(p.vertical_fov_degrees) * Real(0.5));
    horizontal_ = u_ * (Real(2) * half_h * aspect * focus_);
    vertical_ = v_ * (Real(2) * half_h * focus_);
    origin_ = p.look_from;
    lower_left_ = origin_ + forward_ * focus_ - horizontal_ * Real(0.5) - vertical_ * Real(0.5);
    lens_radius_ = p.aperture * Real(0.5);
}

Ray Camera::generate_ray(Vec2 film, Vec2 size, Vec2 lens) const {
    const Real s = film.x / size.x;
    const Real t = Real(1) - film.y / size.y;
    const Vec3 target = lower_left_ + horizontal_ * s + vertical_ * t;
    Vec3 origin = origin_;
    if (lens_radius_ > 0) {
        const Vec2 d = sample_disk_concentric(lens);
        origin += u_ * (d.x * lens_radius_) + v_ * (d.y * lens_radius_);
    }
    return {origin, normalize(target - origin)};
}

}  // namespace lumen
