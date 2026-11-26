// Small linear-algebra toolkit: Vec2/Vec3, 3x3 matrices, affine transforms
// and an orthonormal basis. Everything is single precision on purpose.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace lumen {

using Real = float;
inline constexpr Real kPi = std::numbers::pi_v<Real>;
inline constexpr Real kInvPi = std::numbers::inv_pi_v<Real>;
inline constexpr Real kInf = std::numeric_limits<Real>::infinity();

struct Vec2 {
    Real x = 0;
    Real y = 0;
};

struct Vec3 {
    Real x = 0;
    Real y = 0;
    Real z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}
    constexpr explicit Vec3(Real v) : x(v), y(v), z(v) {}

    constexpr Real operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr Real& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3& operator+=(const Vec3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    constexpr Vec3& operator-=(const Vec3& o) {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }
    constexpr Vec3& operator*=(Real s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    constexpr Vec3& operator*=(const Vec3& o) {
        x *= o.x;
        y *= o.y;
        z *= o.z;
        return *this;
    }
};

constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator*(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
constexpr Vec3 operator*(const Vec3& a, Real s) { return {a.x * s, a.y * s, a.z * s}; }
constexpr Vec3 operator*(Real s, const Vec3& a) { return a * s; }
constexpr Vec3 operator/(const Vec3& a, Real s) {
    const Real inv = Real(1) / s;
    return a * inv;
}
constexpr Vec3 operator/(const Vec3& a, const Vec3& b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }

constexpr Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
constexpr Real length_squared(const Vec3& a) { return dot(a, a); }
inline Real length(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(const Vec3& a) { return a / length(a); }
constexpr Vec3 lerp(const Vec3& a, const Vec3& b, Real t) { return a * (Real(1) - t) + b * t; }
constexpr Real lerp(Real a, Real b, Real t) { return a * (Real(1) - t) + b * t; }
inline Vec3 vmin(const Vec3& a, const Vec3& b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline Vec3 vmax(const Vec3& a, const Vec3& b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
inline Vec3 vabs(const Vec3& a) { return {std::fabs(a.x), std::fabs(a.y), std::fabs(a.z)}; }
inline Real max_component(const Vec3& a) { return std::max({a.x, a.y, a.z}); }
inline Real min_component(const Vec3& a) { return std::min({a.x, a.y, a.z}); }
inline Vec3 vsqrt(const Vec3& a) { return {std::sqrt(a.x), std::sqrt(a.y), std::sqrt(a.z)}; }
inline Vec3 vexp(const Vec3& a) { return {std::exp(a.x), std::exp(a.y), std::exp(a.z)}; }
constexpr Real luminance(const Vec3& c) { return Real(0.2126) * c.x + Real(0.7152) * c.y + Real(0.0722) * c.z; }
inline bool is_finite(const Vec3& a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
constexpr Real sqr(Real v) { return v * v; }
constexpr Real clamp01(Real v) { return v < 0 ? Real(0) : (v > 1 ? Real(1) : v); }
constexpr Real safe_sqrt(Real v) { return v > 0 ? std::sqrt(v) : Real(0); }
constexpr Real radians(Real deg) { return deg * (kPi / Real(180)); }
constexpr Vec3 reflect(const Vec3& v, const Vec3& n) { return v - n * (Real(2) * dot(v, n)); }

/// Row-major 3x3 matrix.
struct Mat3 {
    Vec3 r0{1, 0, 0};
    Vec3 r1{0, 1, 0};
    Vec3 r2{0, 0, 1};

    constexpr Vec3 operator*(const Vec3& v) const { return {dot(r0, v), dot(r1, v), dot(r2, v)}; }
    constexpr Mat3 transposed() const {
        return {{r0.x, r1.x, r2.x}, {r0.y, r1.y, r2.y}, {r0.z, r1.z, r2.z}};
    }
    constexpr Real determinant() const { return dot(r0, cross(r1, r2)); }
};

constexpr Mat3 operator*(const Mat3& a, const Mat3& b) {
    const Mat3 bt = b.transposed();
    return {{dot(a.r0, bt.r0), dot(a.r0, bt.r1), dot(a.r0, bt.r2)},
            {dot(a.r1, bt.r0), dot(a.r1, bt.r1), dot(a.r1, bt.r2)},
            {dot(a.r2, bt.r0), dot(a.r2, bt.r1), dot(a.r2, bt.r2)}};
}

constexpr Mat3 operator*(const Mat3& m, Real s) { return {m.r0 * s, m.r1 * s, m.r2 * s}; }

/// Inverse via the adjugate. The caller guarantees a non-singular matrix.
constexpr Mat3 inverse(const Mat3& m) {
    const Vec3 c0 = cross(m.r1, m.r2);
    const Vec3 c1 = cross(m.r2, m.r0);
    const Vec3 c2 = cross(m.r0, m.r1);
    const Real inv_det = Real(1) / dot(m.r0, c0);
    return Mat3{c0, c1, c2}.transposed() * inv_det;
}

/// Affine transform: scale, then rotate (X, Y, Z order), then translate.
class Transform {
public:
    Transform() = default;
    static Transform from_trs(const Vec3& translate, const Vec3& rotate_degrees, const Vec3& scale);

    Vec3 point(const Vec3& p) const { return linear_ * p + translation_; }
    Vec3 vector(const Vec3& v) const { return linear_ * v; }
    /// Normals transform with the inverse transpose so they stay perpendicular.
    Vec3 normal(const Vec3& n) const { return normalize(normal_matrix_ * n); }
    /// A negative determinant mirrors the mesh and therefore flips winding.
    bool flips_winding() const { return linear_.determinant() < 0; }

private:
    Mat3 linear_{};
    Mat3 normal_matrix_{};
    Vec3 translation_{};
};

/// Orthonormal basis around a unit normal (branchless construction by Duff et al.).
struct Onb {
    Vec3 t;
    Vec3 b;
    Vec3 n;

    static Onb from_normal(const Vec3& n) {
        const Real sign = std::copysign(Real(1), n.z);
        const Real a = Real(-1) / (sign + n.z);
        const Real bb = n.x * n.y * a;
        return {Vec3(Real(1) + sign * n.x * n.x * a, sign * bb, -sign * n.x),
                Vec3(bb, sign + n.y * n.y * a, -n.y), n};
    }
    Vec3 to_local(const Vec3& v) const { return {dot(v, t), dot(v, b), dot(v, n)}; }
    Vec3 to_world(const Vec3& v) const { return t * v.x + b * v.y + n * v.z; }
};

}  // namespace lumen
