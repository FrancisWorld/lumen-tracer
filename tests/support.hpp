// Helpers shared by the tests.
#pragma once

#include <cmath>
#include <functional>
#include <numbers>

#include "lumen/math.hpp"

namespace lumen::testing {


/// Midpoint quadrature of f over the whole unit sphere in solid-angle measure.
inline double integrate_sphere(const std::function<double(const Vec3&)>& f, int n_theta = 400, int n_phi = 800) {
    double sum = 0;
    for (int i = 0; i < n_theta; ++i) {
        const double theta = (i + 0.5) / n_theta * std::numbers::pi;
        for (int j = 0; j < n_phi; ++j) {
            const double phi = (j + 0.5) / n_phi * 2.0 * std::numbers::pi;
            const Vec3 w(static_cast<Real>(std::sin(theta) * std::cos(phi)),
                         static_cast<Real>(std::sin(theta) * std::sin(phi)), static_cast<Real>(std::cos(theta)));
            sum += f(w) * std::sin(theta);
        }
    }
    return sum * (std::numbers::pi / n_theta) * (2.0 * std::numbers::pi / n_phi);
}


}  // namespace lumen::testing
