// Bounding volume hierarchy built with the binned surface area heuristic.
// The BVH is geometry-agnostic: it orders primitive indices and calls back
// into a leaf test, so the same code is tested against brute force.
#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "lumen/geometry.hpp"

namespace lumen {

struct BvhStats {
    std::size_t node_count = 0;
    std::size_t leaf_count = 0;
    std::size_t max_depth = 0;
    std::size_t max_leaf_size = 0;
    /// Expected traversal cost under the SAH model (lower is better).
    double sah_cost = 0;
};

class Bvh {
public:
    struct Node {
        Aabb box;
        // Leaf: first index into order(). Interior: index of the second child
        // (the first child always directly follows its parent).
        std::uint32_t offset = 0;
        std::uint16_t count = 0;  // 0 for interior nodes
        std::uint8_t axis = 0;
    };

    struct BuildOptions {
        std::uint32_t max_leaf_size = 4;
        std::uint32_t bin_count = 16;
    };

    Bvh() = default;
    static Bvh build(std::span<const Aabb> bounds, BuildOptions options);
    static Bvh build(std::span<const Aabb> bounds) { return build(bounds, BuildOptions{}); }

    std::span<const Node> nodes() const { return nodes_; }
    /// Primitive indices in leaf order.
    std::span<const std::uint32_t> order() const { return order_; }
    BvhStats stats() const;

    /// Visit leaf primitives front to back. `test(prim, t_max)` returns true on
    /// a hit and must lower `t_max` to the new closest distance. With
    /// `any_hit` the traversal stops at the first hit. Returns true on any hit.
    template <typename LeafTest>
    bool traverse(const Ray& ray, Real& t_max, bool any_hit, LeafTest&& test) const {
        if (nodes_.empty()) {
            return false;
        }
        const Vec3 inv_dir(Real(1) / ray.direction.x, Real(1) / ray.direction.y, Real(1) / ray.direction.z);
        const bool negative[3] = {inv_dir.x < 0, inv_dir.y < 0, inv_dir.z < 0};
        std::uint32_t stack[kStackSize];
        int top = 0;
        std::uint32_t current = 0;
        bool found = false;
        for (;;) {
            const Node& node = nodes_[current];
            if (node.box.hit(ray.origin, inv_dir, t_max)) {
                if (node.count > 0) {
                    for (std::uint32_t i = 0; i < node.count; ++i) {
                        if (test(order_[node.offset + i], t_max)) {
                            found = true;
                            if (any_hit) {
                                return true;
                            }
                        }
                    }
                    if (top == 0) {
                        break;
                    }
                    current = stack[--top];
                } else if (negative[node.axis]) {
                    stack[top++] = current + 1;
                    current = node.offset;
                } else {
                    stack[top++] = node.offset;
                    current = current + 1;
                }
            } else {
                if (top == 0) {
                    break;
                }
                current = stack[--top];
            }
        }
        return found;
    }

private:
    static constexpr int kStackSize = 128;
    std::vector<Node> nodes_;
    std::vector<std::uint32_t> order_;
};

}  // namespace lumen
