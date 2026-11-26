#include "lumen/camera.hpp"
#include "lumen/rng.hpp"
#include "testing.hpp"

using namespace lumen;

TEST_CASE(camera, pinhole_centre_ray_looks_at_target) {
    CameraParams p;
    p.look_from = {1, 2, 3};
    p.look_at = {1, 2, -4};
    const Camera cam(p, 16.0F / 9.0F);
    const Ray r = cam.generate_ray({160, 90}, {320, 180}, {0.5F, 0.5F});
    CHECK_NEAR(r.direction.z, -1.0, 1e-5);
    CHECK_NEAR(length(r.direction), 1.0, 1e-5);
    CHECK_NEAR(r.origin.x, 1.0, 0);
}

TEST_CASE(camera, field_of_view_and_aspect_define_the_frustum) {
    CameraParams p;
    p.look_from = {0, 0, 0};
    p.look_at = {0, 0, -1};
    p.vertical_fov_degrees = 90;
    const Camera cam(p, 2.0F);
    const Vec2 size{200, 100};
    const Ray top = cam.generate_ray({100, 0}, size, {});
    CHECK_NEAR(top.direction.y / -top.direction.z, 1.0, 1e-5);  // tan(45 deg)
    const Ray right = cam.generate_ray({200, 50}, size, {});
    CHECK_NEAR(right.direction.x / -right.direction.z, 2.0, 1e-5);  // aspect 2 widens the horizontal extent
    const Ray left = cam.generate_ray({0, 50}, size, {});
    CHECK(left.direction.x < 0);
}

TEST_CASE(camera, thin_lens_rays_converge_on_the_focal_plane) {
    CameraParams p;
    p.look_from = {0, 1, 6};
    p.look_at = {0, 0, 0};
    p.aperture = 0.5F;
    p.focus_distance = 4;
    const Camera cam(p, 1.5F);
    const Vec2 size{300, 200};
    Pcg32 rng(2);
    for (const Vec2 pixel : {Vec2{150, 100}, Vec2{20, 30}, Vec2{280, 170}}) {
        const Vec3 first = [&] {
            const Ray r = cam.generate_ray(pixel, size, {0.5F, 0.5F});
            return r.at(cam.focus_distance() / dot(r.direction, cam.forward()));
        }();
        bool distinct_origins = false;
        for (int i = 0; i < 200; ++i) {
            const Ray r = cam.generate_ray(pixel, size, rng.next_vec2());
            const Vec3 on_plane = r.at(cam.focus_distance() / dot(r.direction, cam.forward()));
            CHECK_NEAR(length(on_plane - first), 0.0, 2e-4);  // in focus: all lens samples meet at one point
            distinct_origins = distinct_origins || length(r.origin - p.look_from) > 0.01F;
            CHECK(length(r.origin - p.look_from) <= 0.25F + 1e-4F);  // origin stays inside the lens
        }
        CHECK(distinct_origins);
    }
}

TEST_CASE(camera, out_of_focus_points_blur) {
    CameraParams p;
    p.look_from = {0, 0, 0};
    p.look_at = {0, 0, -1};
    p.aperture = 0.2F;
    p.focus_distance = 2;
    const Camera cam(p, 1.0F);
    Pcg32 rng(4);
    // A point at depth 5 seen through the pixel centre: its image spreads over the film (circle of confusion).
    double spread = 0;
    for (int i = 0; i < 100; ++i) {
        const Ray r = cam.generate_ray({50, 50}, {100, 100}, rng.next_vec2());
        const Vec3 at_depth = r.at(5.0F / -r.direction.z);
        spread = std::max<double>(spread, length(at_depth));
    }
    CHECK(spread > 0.1);
}

TEST_CASE(camera, rejects_degenerate_parameters) {
    CameraParams same;
    same.look_from = same.look_at;
    CHECK_THROWS(Camera(same, 1));
    CameraParams up;
    up.look_from = {0, 0, 0};
    up.look_at = {0, 5, 0};  // looking straight along the up vector
    CHECK_THROWS(Camera(up, 1));
    CameraParams fov;
    fov.vertical_fov_degrees = 0;
    CHECK_THROWS(Camera(fov, 1));
    CameraParams ok;
    CHECK_THROWS(Camera(ok, 0));
    ok.aperture = -1;
    CHECK_THROWS(Camera(ok, 1));
}
