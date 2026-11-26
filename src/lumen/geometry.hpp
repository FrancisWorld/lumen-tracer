// Rays, bounding boxes and the two primitive types (sphere, triangle).
#pragma once

#include <cstdint>

#include "lumen/math.hpp"

namespace lumen {

struct Ray {
    Vec3 origin;
    Vec3 direction;  // unit length
    Vec3 at(Real t) const { return origin + direction * t; }
};

struct Aabb {
    Vec3 lo{kInf, kInf, kInf};
    Vec3 hi{-kInf, -kInf, -kInf};

    void grow(const Vec3& p) {
        lo = vmin(lo, p);
        hi = vmax(hi, p);
    }
    void grow(const Aabb& b) {
        lo = vmin(lo, b.lo);
        hi = vmax(hi, b.hi);
    }
    bool empty() const { return lo.x > hi.x || lo.y > hi.y || lo.z > hi.z; }
    Vec3 extent() const { return hi - lo; }
    Vec3 centroid() const { return (lo + hi) * Real(0.5); }
    Real surface_area() const {
        if (empty()) {
            return 0;
        }
        const Vec3 e = extent();
        return Real(2) * (e.x * e.y + e.y * e.z + e.z * e.x);
    }
    int longest_axis() const {
        const Vec3 e = extent();
        return (e.x > e.y && e.x > e.z) ? 0 : (e.y > e.z ? 1 : 2);
    }
    bool contains(const Aabb& b, Real eps = 0) const {
        return b.lo.x >= lo.x - eps && b.lo.y >= lo.y - eps && b.lo.z >= lo.z - eps &&
               b.hi.x <= hi.x + eps && b.hi.y <= hi.y + eps && b.hi.z <= hi.z + eps;
    }

    /// Slab test using the precomputed reciprocal direction. Handles zero direction components.
    bool hit(const Vec3& origin, const Vec3& inv_dir, Real t_max) const {
        Real t0 = 0;
        Real t1 = t_max;
        for (int a = 0; a < 3; ++a) {
            Real tn = (lo[a] - origin[a]) * inv_dir[a];
            Real tf = (hi[a] - origin[a]) * inv_dir[a];
            if (tn > tf) {
                std::swap(tn, tf);
            }
            // NaN (0 * inf) compares false and leaves the interval untouched.
            t0 = tn > t0 ? tn : t0;
            t1 = tf < t1 ? tf : t1;
            if (t0 > t1) {
                return false;
            }
        }
        return true;
    }
};

struct Sphere {
    Vec3 center;
    Real radius = 1;
    std::uint32_t material = 0;

    Aabb bounds() const { return {center - Vec3(radius), center + Vec3(radius)}; }

    /// Nearest root in (0, t_max). Uses the cancellation-free formulation from
    /// "Precision Improvements for Ray/Sphere Intersection" (Haines et al.).
    bool intersect(const Ray& ray, Real t_max, Real& t) const {
        const Vec3 oc = ray.origin - center;
        const Real b = dot(oc, ray.direction);
        const Vec3 h = oc - ray.direction * b;
        const Real disc = radius * radius - length_squared(h);
        if (disc < 0) {
            return false;
        }
        const Real sq = std::sqrt(disc);
        // q has the same sign as -b, so no catastrophic cancellation in either root.
        const Real q = b > 0 ? -b - sq : -b + sq;
        const Real c = length_squared(oc) - radius * radius;
        Real t0 = q;
        Real t1 = q != 0 ? c / q : q;
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        if (t0 > 0 && t0 < t_max) {
            t = t0;
            return true;
        }
        if (t1 > 0 && t1 < t_max) {
            t = t1;
            return true;
        }
        return false;
    }

    Real area() const { return Real(4) * kPi * radius * radius; }
};

struct Triangle {
    Vec3 p0, p1, p2;
    Vec3 n0, n1, n2;  // per-vertex normals (equal to the face normal when flat)
    Vec2 uv0, uv1, uv2;
    std::uint32_t material = 0;

    Aabb bounds() const {
        Aabb b;
        b.grow(p0);
        b.grow(p1);
        b.grow(p2);
        return b;
    }
    Vec3 face_normal() const { return normalize(cross(p1 - p0, p2 - p0)); }
    Real area() const { return Real(0.5) * length(cross(p1 - p0, p2 - p0)); }

    /// Moeller-Trumbore. Returns barycentrics (u, v) of p1 and p2.
    bool intersect(const Ray& ray, Real t_max, Real& t, Real& u, Real& v) const {
        const Vec3 e1 = p1 - p0;
        const Vec3 e2 = p2 - p0;
        const Vec3 pv = cross(ray.direction, e2);
        const Real det = dot(e1, pv);
        if (std::fabs(det) < Real(1e-12)) {
            return false;
        }
        const Real inv = Real(1) / det;
        const Vec3 tv = ray.origin - p0;
        const Real uu = dot(tv, pv) * inv;
        if (uu < 0 || uu > 1) {
            return false;
        }
        const Vec3 qv = cross(tv, e1);
        const Real vv = dot(ray.direction, qv) * inv;
        if (vv < 0 || uu + vv > 1) {
            return false;
        }
        const Real tt = dot(e2, qv) * inv;
        if (tt <= 0 || tt >= t_max) {
            return false;
        }
        t = tt;
        u = uu;
        v = vv;
        return true;
    }
};

}  // namespace lumen
