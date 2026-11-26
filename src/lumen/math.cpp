#include "lumen/math.hpp"

namespace lumen {

Transform Transform::from_trs(const Vec3& translate, const Vec3& rotate_degrees, const Vec3& scale) {
    const Real cx = std::cos(radians(rotate_degrees.x)), sx = std::sin(radians(rotate_degrees.x));
    const Real cy = std::cos(radians(rotate_degrees.y)), sy = std::sin(radians(rotate_degrees.y));
    const Real cz = std::cos(radians(rotate_degrees.z)), sz = std::sin(radians(rotate_degrees.z));
    const Mat3 rx{{1, 0, 0}, {0, cx, -sx}, {0, sx, cx}};
    const Mat3 ry{{cy, 0, sy}, {0, 1, 0}, {-sy, 0, cy}};
    const Mat3 rz{{cz, -sz, 0}, {sz, cz, 0}, {0, 0, 1}};
    const Mat3 s{{scale.x, 0, 0}, {0, scale.y, 0}, {0, 0, scale.z}};
    Transform t;
    t.linear_ = rz * ry * rx * s;
    t.normal_matrix_ = inverse(t.linear_).transposed();
    t.translation_ = translate;
    return t;
}

}  // namespace lumen
