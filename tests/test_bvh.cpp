#include <algorithm>
#include <numeric>
#include <vector>

#include "lumen/bvh.hpp"
#include "lumen/rng.hpp"
#include "testing.hpp"

using namespace lumen;

namespace {

struct Soup {
    std::vector<Sphere> spheres;
    std::vector<Triangle> triangles;
    std::vector<Aabb> bounds;  // spheres first, then triangles

    std::size_t size() const { return bounds.size(); }

    bool test(std::uint32_t prim, const Ray& ray, Real& t_max) const {
        Real t = 0;
        Real u = 0;
        Real v = 0;
        bool hit = false;
        if (prim < spheres.size()) {
            hit = spheres[prim].intersect(ray, t_max, t);
        } else {
            hit = triangles[prim - spheres.size()].intersect(ray, t_max, t, u, v);
        }
        if (hit) {
            t_max = t;
        }
        return hit;
    }
};

Vec3 random_point(Pcg32& rng, Real extent) {
    return {(rng.next_real() - 0.5F) * extent, (rng.next_real() - 0.5F) * extent, (rng.next_real() - 0.5F) * extent};
}

Soup make_soup(std::uint64_t seed, int spheres, int triangles) {
    Pcg32 rng(seed);
    Soup s;
    for (int i = 0; i < spheres; ++i) {
        s.spheres.push_back({random_point(rng, 20), 0.2F + rng.next_real() * 0.6F, 0});
    }
    for (int i = 0; i < triangles; ++i) {
        const Vec3 c = random_point(rng, 20);
        Triangle t;
        t.p0 = c + random_point(rng, 2);
        t.p1 = c + random_point(rng, 2);
        t.p2 = c + random_point(rng, 2);
        s.triangles.push_back(t);
    }
    for (const Sphere& sp : s.spheres) {
        s.bounds.push_back(sp.bounds());
    }
    for (const Triangle& t : s.triangles) {
        s.bounds.push_back(t.bounds());
    }
    return s;
}

Ray random_ray(Pcg32& rng) {
    const Vec3 origin = random_point(rng, 30);
    Vec3 d = random_point(rng, 2);
    if (length_squared(d) < 1e-4F) {
        d = {0, 0, 1};
    }
    return {origin, normalize(d)};
}

struct Closest {
    bool hit = false;
    Real t = kInf;
};

Closest brute_force(const Soup& s, const Ray& ray) {
    Closest c;
    Real t_max = kInf;
    for (std::uint32_t i = 0; i < s.size(); ++i) {
        if (s.test(i, ray, t_max)) {
            c.hit = true;
            c.t = t_max;
        }
    }
    return c;
}

}  // namespace

TEST_CASE(bvh, closest_hit_matches_brute_force) {
    const Soup soup = make_soup(1, 150, 350);
    const Bvh bvh = Bvh::build(soup.bounds);
    Pcg32 rng(99);
    int hits = 0;
    for (int i = 0; i < 4000; ++i) {
        const Ray ray = random_ray(rng);
        const Closest expected = brute_force(soup, ray);
        Real t_max = kInf;
        const bool hit = bvh.traverse(ray, t_max, false,
                                      [&](std::uint32_t p, Real& tm) { return soup.test(p, ray, tm); });
        REQUIRE(hit == expected.hit);
        if (hit) {
            ++hits;
            CHECK_EQ(t_max, expected.t);
        }
    }
    CHECK(hits > 300);  // the test must actually exercise hits, not only misses
}

TEST_CASE(bvh, any_hit_agrees_with_closest_hit_existence) {
    const Soup soup = make_soup(2, 100, 200);
    const Bvh bvh = Bvh::build(soup.bounds);
    Pcg32 rng(7);
    for (int i = 0; i < 2000; ++i) {
        const Ray ray = random_ray(rng);
        Real t_a = kInf;
        const bool any = bvh.traverse(ray, t_a, true, [&](std::uint32_t p, Real& tm) { return soup.test(p, ray, tm); });
        CHECK_EQ(any, brute_force(soup, ray).hit);
    }
}

TEST_CASE(bvh, respects_t_max) {
    const Soup soup = make_soup(3, 60, 60);
    const Bvh bvh = Bvh::build(soup.bounds);
    Pcg32 rng(17);
    for (int i = 0; i < 1000; ++i) {
        const Ray ray = random_ray(rng);
        const Closest full = brute_force(soup, ray);
        if (!full.hit) {
            continue;
        }
        Real limit = full.t * 0.5F;
        const bool hit = bvh.traverse(ray, limit, false, [&](std::uint32_t p, Real& tm) { return soup.test(p, ray, tm); });
        CHECK(!hit || limit < full.t * 0.5F + 1e-6F);
        CHECK(!hit);  // closest surface lies beyond the limit, so nothing may be reported
    }
}

TEST_CASE(bvh, structure_is_a_valid_partition) {
    const Soup soup = make_soup(4, 200, 300);
    const Bvh bvh = Bvh::build(soup.bounds, {.max_leaf_size = 4, .bin_count = 16});
    // Every primitive appears exactly once in the leaf order.
    std::vector<std::uint32_t> sorted(bvh.order().begin(), bvh.order().end());
    std::sort(sorted.begin(), sorted.end());
    std::vector<std::uint32_t> expected(soup.size());
    std::iota(expected.begin(), expected.end(), 0U);
    CHECK(sorted == expected);

    // Parent boxes contain child boxes; leaves contain their primitives.
    const auto nodes = bvh.nodes();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const Bvh::Node& n = nodes[i];
        if (n.count == 0) {
            CHECK(n.box.contains(nodes[i + 1].box, 1e-4F));
            CHECK(n.box.contains(nodes[n.offset].box, 1e-4F));
        } else {
            for (std::uint32_t k = 0; k < n.count; ++k) {
                CHECK(n.box.contains(soup.bounds[bvh.order()[n.offset + k]], 1e-4F));
            }
        }
    }
    const BvhStats stats = bvh.stats();
    CHECK(stats.max_leaf_size <= 4);
    CHECK_EQ(stats.node_count, 2 * stats.leaf_count - 1);
    CHECK(stats.max_depth < 40);
}

TEST_CASE(bvh, sah_beats_a_naive_median_split_on_clustered_scenes) {
    // Two distant clusters: a good SAH build separates them at the root.
    Soup soup;
    Pcg32 rng(5);
    for (int i = 0; i < 200; ++i) {
        const Real cx = i % 2 == 0 ? -100.0F : 100.0F;
        soup.spheres.push_back({Vec3(cx, 0, 0) + random_point(rng, 4), 0.3F, 0});
    }
    for (const Sphere& s : soup.spheres) {
        soup.bounds.push_back(s.bounds());
    }
    const Bvh bvh = Bvh::build(soup.bounds);
    const auto nodes = bvh.nodes();
    CHECK(nodes[1].box.hi.x < 0 || nodes[1].box.lo.x > 0);
    CHECK(nodes[nodes[0].offset].box.hi.x < 0 || nodes[nodes[0].offset].box.lo.x > 0);
    // Cost relative to a flat list (every primitive tested): must be far lower.
    CHECK(bvh.stats().sah_cost < 0.2 * 2.0 * static_cast<double>(soup.size()));
}

TEST_CASE(bvh, degenerate_inputs) {
    CHECK(Bvh::build({}).nodes().empty());
    Real t_max = kInf;
    CHECK(!Bvh{}.traverse({{0, 0, 0}, {0, 0, 1}}, t_max, false, [](std::uint32_t, Real&) { return true; }));

    // Many primitives with identical centroids cannot be split spatially.
    std::vector<Aabb> same(40, Aabb{{-1, -1, -1}, {1, 1, 1}});
    const Bvh bvh = Bvh::build(same, {.max_leaf_size = 4, .bin_count = 16});
    CHECK(bvh.stats().max_leaf_size <= 4);
    CHECK_EQ(bvh.order().size(), same.size());

    // Single primitive, and rays with axis-aligned (zero) direction components.
    Soup one;
    one.spheres.push_back({{0, 0, 5}, 1, 0});
    one.bounds.push_back(one.spheres[0].bounds());
    const Bvh single = Bvh::build(one.bounds);
    const Ray along_z{{0, 0, 0}, {0, 0, 1}};
    Real t = kInf;
    CHECK(single.traverse(along_z, t, false, [&](std::uint32_t p, Real& tm) { return one.test(p, along_z, tm); }));
    CHECK_NEAR(t, 4.0, 1e-4);
}
