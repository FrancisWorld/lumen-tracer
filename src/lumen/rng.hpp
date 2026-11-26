// PCG32 generator plus the hashing used to derive one independent stream per
// pixel. Seeding by (seed, pixel) instead of by thread is what makes renders
// bit-identical regardless of thread count or tile order.
#pragma once

#include <cstdint>

#include "lumen/math.hpp"

namespace lumen {

/// SplitMix64 finaliser: a cheap, well-mixed 64-bit hash.
constexpr std::uint64_t mix64(std::uint64_t z) {
    z += 0x9e3779b97f4a7c15ULL;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

constexpr std::uint64_t hash_combine(std::uint64_t a, std::uint64_t b) { return mix64(a ^ mix64(b)); }

class Pcg32 {
public:
    constexpr explicit Pcg32(std::uint64_t seed, std::uint64_t stream = 0) {
        inc_ = (stream << 1U) | 1U;
        next_u32();
        state_ += seed;
        next_u32();
    }

    constexpr std::uint32_t next_u32() {
        const std::uint64_t old = state_;
        state_ = old * 6364136223846793005ULL + inc_;
        const auto xorshifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
        const auto rot = static_cast<std::uint32_t>(old >> 59U);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1U) & 31U));
    }

    /// Uniform in [0, 1): 24 random bits so every value is exactly representable.
    constexpr Real next_real() { return static_cast<Real>(next_u32() >> 8U) * (Real(1) / Real(16777216)); }
    constexpr Vec2 next_vec2() {
        const Real a = next_real();
        return {a, next_real()};
    }

private:
    std::uint64_t state_ = 0;
    std::uint64_t inc_ = 1;
};

/// Independent generator for one pixel of one render.
constexpr Pcg32 pixel_rng(std::uint64_t seed, std::uint32_t x, std::uint32_t y) {
    return Pcg32(hash_combine(seed, (static_cast<std::uint64_t>(y) << 32U) | x));
}

}  // namespace lumen
