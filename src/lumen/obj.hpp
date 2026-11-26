// Wavefront OBJ reader: v/vt/vn/f with polygons (fan triangulated), negative
// indices and 'v/vt/vn' index forms. Other statements are ignored.
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "lumen/geometry.hpp"

namespace lumen {

struct ObjVertex {
    Vec3 position;
    Vec3 normal;  // zero length when the file supplies none
    Vec2 uv;
    bool has_normal = false;
};

struct ObjMesh {
    std::vector<ObjVertex> vertices;
    std::vector<std::uint32_t> indices;  // three per triangle
    std::size_t triangle_count() const { return indices.size() / 3; }
};

/// Throws InputError("obj: line N: ...") on malformed input.
ObjMesh parse_obj(std::string_view text);
ObjMesh load_obj_file(const std::string& path);

}  // namespace lumen
