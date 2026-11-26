#include "lumen/bvh.hpp"

#include <algorithm>
#include <array>
#include <numeric>
#include <string>

#include "lumen/error.hpp"

namespace lumen {
namespace {

constexpr double kTraversalCost = 1.0;
constexpr double kIntersectionCost = 2.0;

struct PrimInfo {
    Aabb box;
    Vec3 centroid;
    std::uint32_t index;
};

class Builder {
public:
    Builder(std::vector<PrimInfo>& prims, Bvh::BuildOptions options, std::vector<Bvh::Node>& nodes)
        : prims_(prims), options_(options), nodes_(nodes) {}

    void build_range(std::size_t begin, std::size_t end) {
        const std::size_t node_index = nodes_.size();
        nodes_.emplace_back();
        Aabb box;
        Aabb centroid_box;
        for (std::size_t i = begin; i < end; ++i) {
            box.grow(prims_[i].box);
            centroid_box.grow(prims_[i].centroid);
        }
        nodes_[node_index].box = box;
        const std::size_t count = end - begin;

        auto make_leaf = [&] {
            nodes_[node_index].offset = static_cast<std::uint32_t>(begin);
            nodes_[node_index].count = static_cast<std::uint16_t>(count);
        };

        if (count == 1) {
            make_leaf();
            return;
        }

        const int axis = centroid_box.longest_axis();
        const Real lo = centroid_box.lo[axis];
        const Real extent = centroid_box.hi[axis] - lo;
        std::size_t mid = begin + count / 2;

        if (extent <= 0) {
            // All centroids coincide: no spatial split exists.
            if (count <= options_.max_leaf_size) {
                make_leaf();
                return;
            }
        } else {
            const std::uint32_t bins = options_.bin_count;
            std::vector<Aabb> bin_box(bins);
            std::vector<std::size_t> bin_count(bins, 0);
            auto bin_of = [&](const PrimInfo& p) {
                const auto b = static_cast<std::uint32_t>((p.centroid[axis] - lo) / extent * Real(bins));
                return std::min(b, bins - 1);
            };
            for (std::size_t i = begin; i < end; ++i) {
                const std::uint32_t b = bin_of(prims_[i]);
                ++bin_count[b];
                bin_box[b].grow(prims_[i].box);
            }
            // Sweep from both ends to get the cost of every split plane.
            std::vector<double> left_cost(bins - 1);
            Aabb acc;
            std::size_t n = 0;
            for (std::uint32_t i = 0; i + 1 < bins; ++i) {
                acc.grow(bin_box[i]);
                n += bin_count[i];
                left_cost[i] = static_cast<double>(acc.surface_area()) * static_cast<double>(n);
            }
            acc = Aabb{};
            n = 0;
            double best_cost = std::numeric_limits<double>::infinity();
            std::uint32_t best_split = 0;
            for (std::uint32_t i = bins - 1; i > 0; --i) {
                acc.grow(bin_box[i]);
                n += bin_count[i];
                const double cost =
                    left_cost[i - 1] + static_cast<double>(acc.surface_area()) * static_cast<double>(n);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_split = i - 1;
                }
            }
            const double parent_area = std::max(static_cast<double>(box.surface_area()), 1e-30);
            const double split_cost = kTraversalCost + kIntersectionCost * best_cost / parent_area;
            const double leaf_cost = kIntersectionCost * static_cast<double>(count);
            if (count <= options_.max_leaf_size && leaf_cost <= split_cost) {
                make_leaf();
                return;
            }
            const auto it = std::partition(prims_.begin() + static_cast<std::ptrdiff_t>(begin),
                                           prims_.begin() + static_cast<std::ptrdiff_t>(end),
                                           [&](const PrimInfo& p) { return bin_of(p) <= best_split; });
            mid = static_cast<std::size_t>(it - prims_.begin());
        }

        if (mid == begin || mid == end) {
            // Degenerate partition: fall back to a median split along the axis.
            mid = begin + count / 2;
            std::nth_element(prims_.begin() + static_cast<std::ptrdiff_t>(begin),
                             prims_.begin() + static_cast<std::ptrdiff_t>(mid),
                             prims_.begin() + static_cast<std::ptrdiff_t>(end),
                             [axis](const PrimInfo& a, const PrimInfo& b) { return a.centroid[axis] < b.centroid[axis]; });
        }

        nodes_[node_index].axis = static_cast<std::uint8_t>(axis);
        build_range(begin, mid);
        nodes_[node_index].offset = static_cast<std::uint32_t>(nodes_.size());
        build_range(mid, end);
    }

private:
    std::vector<PrimInfo>& prims_;
    Bvh::BuildOptions options_;
    std::vector<Bvh::Node>& nodes_;
};

}  // namespace

Bvh Bvh::build(std::span<const Aabb> bounds, BuildOptions options) {
    Bvh bvh;
    if (bounds.empty()) {
        return bvh;
    }
    options.max_leaf_size = std::clamp<std::uint32_t>(options.max_leaf_size, 1, 255);
    options.bin_count = std::clamp<std::uint32_t>(options.bin_count, 2, 64);
    std::vector<PrimInfo> prims(bounds.size());
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        // Non-finite boxes would turn centroids into NaN and make the bin index conversion undefined.
        if (!is_finite(bounds[i].lo) || !is_finite(bounds[i].hi) || bounds[i].empty() ||
            !is_finite(bounds[i].hi - bounds[i].lo)) {
            throw InputError("bvh: primitive " + std::to_string(i) + " has an empty or non-finite bounding box");
        }
        prims[i] = {bounds[i], bounds[i].centroid(), static_cast<std::uint32_t>(i)};
    }
    bvh.nodes_.reserve(bounds.size() * 2);
    Builder(prims, options, bvh.nodes_).build_range(0, prims.size());
    bvh.order_.resize(prims.size());
    for (std::size_t i = 0; i < prims.size(); ++i) {
        bvh.order_[i] = prims[i].index;
    }
    return bvh;
}

BvhStats Bvh::stats() const {
    BvhStats s;
    s.node_count = nodes_.size();
    if (nodes_.empty()) {
        return s;
    }
    const double root_area = std::max(static_cast<double>(nodes_[0].box.surface_area()), 1e-30);
    struct Item {
        std::uint32_t node;
        std::size_t depth;
    };
    std::vector<Item> stack{{0, 1}};
    while (!stack.empty()) {
        const auto [index, depth] = stack.back();
        stack.pop_back();
        const Node& n = nodes_[index];
        s.max_depth = std::max(s.max_depth, depth);
        const double rel_area = static_cast<double>(n.box.surface_area()) / root_area;
        if (n.count > 0) {
            ++s.leaf_count;
            s.max_leaf_size = std::max<std::size_t>(s.max_leaf_size, n.count);
            s.sah_cost += rel_area * kIntersectionCost * n.count;
        } else {
            s.sah_cost += rel_area * kTraversalCost;
            stack.push_back({index + 1, depth + 1});
            stack.push_back({n.offset, depth + 1});
        }
    }
    return s;
}

}  // namespace lumen
