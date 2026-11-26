#include "lumen/obj.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <tuple>

#include "lumen/error.hpp"

namespace lumen {
namespace {

class LineParser {
public:
    LineParser(std::string_view line, std::size_t number) : rest_(line), number_(number) {}

    [[noreturn]] void fail(const std::string& what) const {
        throw InputError("obj: line " + std::to_string(number_) + ": " + what);
    }

    std::string_view next_token() {
        const auto start = rest_.find_first_not_of(" \t\r");
        if (start == std::string_view::npos) {
            rest_ = {};
            return {};
        }
        rest_.remove_prefix(start);
        const auto end = rest_.find_first_of(" \t\r");
        const auto token = rest_.substr(0, end);
        rest_ = end == std::string_view::npos ? std::string_view{} : rest_.substr(end);
        return token;
    }

    Real next_real() {
        const auto t = next_token();
        Real v = 0;
        const auto [ptr, ec] = std::from_chars(t.data(), t.data() + t.size(), v);
        if (t.empty() || ec != std::errc() || ptr != t.data() + t.size() || !std::isfinite(v)) {
            fail("expected a number, found '" + std::string(t) + "'");
        }
        return v;
    }

private:
    std::string_view rest_;
    std::size_t number_;
};

struct FaceRef {
    long v = 0;
    long vt = 0;
    long vn = 0;
};

}  // namespace

ObjMesh parse_obj(std::string_view text) {
    std::vector<Vec3> positions;
    std::vector<Vec2> uvs;
    std::vector<Vec3> normals;
    ObjMesh mesh;
    std::map<std::tuple<long, long, long>, std::uint32_t> dedupe;

    auto resolve = [](long index, std::size_t count, const LineParser& lp) -> long {
        // OBJ indices are 1-based; negative values count back from the end.
        const long resolved = index > 0 ? index - 1 : static_cast<long>(count) + index;
        if (index == 0 || resolved < 0 || resolved >= static_cast<long>(count)) {
            lp.fail("index " + std::to_string(index) + " out of range");
        }
        return resolved;
    };

    std::size_t line_no = 0;
    std::size_t pos = 0;
    while (pos <= text.size()) {
        auto eol = text.find('\n', pos);
        if (eol == std::string_view::npos) {
            eol = text.size();
        }
        std::string_view line = text.substr(pos, eol - pos);
        pos = eol + 1;
        ++line_no;
        if (const auto hash = line.find('#'); hash != std::string_view::npos) {
            line = line.substr(0, hash);
        }
        LineParser lp(line, line_no);
        const auto keyword = lp.next_token();
        if (keyword.empty()) {
            continue;
        }
        if (keyword == "v") {
            const Real x = lp.next_real();
            const Real y = lp.next_real();
            positions.emplace_back(x, y, lp.next_real());
        } else if (keyword == "vt") {
            const Real u = lp.next_real();
            uvs.push_back({u, lp.next_real()});
        } else if (keyword == "vn") {
            const Real x = lp.next_real();
            const Real y = lp.next_real();
            normals.emplace_back(x, y, lp.next_real());
        } else if (keyword == "f") {
            std::vector<std::uint32_t> face;
            for (auto token = lp.next_token(); !token.empty(); token = lp.next_token()) {
                FaceRef ref;
                std::array<long*, 3> slots = {&ref.v, &ref.vt, &ref.vn};
                std::size_t slot = 0;
                std::size_t start = 0;
                while (start <= token.size()) {
                    auto slash = token.find('/', start);
                    if (slash == std::string_view::npos) {
                        slash = token.size();
                    }
                    const auto part = token.substr(start, slash - start);
                    if (slot >= 3) {
                        lp.fail("too many components in '" + std::string(token) + "'");
                    }
                    if (!part.empty()) {
                        const auto [ptr, ec] = std::from_chars(part.data(), part.data() + part.size(), *slots[slot]);
                        if (ec != std::errc() || ptr != part.data() + part.size()) {
                            lp.fail("bad face index '" + std::string(token) + "'");
                        }
                    }
                    ++slot;
                    start = slash + 1;
                }
                if (ref.v == 0) {
                    lp.fail("face vertex without a position");
                }
                const long vi = resolve(ref.v, positions.size(), lp);
                const long ti = ref.vt != 0 ? resolve(ref.vt, uvs.size(), lp) : -1;
                const long ni = ref.vn != 0 ? resolve(ref.vn, normals.size(), lp) : -1;
                const auto key = std::make_tuple(vi, ti, ni);
                auto [it, inserted] = dedupe.try_emplace(key, static_cast<std::uint32_t>(mesh.vertices.size()));
                if (inserted) {
                    ObjVertex v;
                    v.position = positions[static_cast<std::size_t>(vi)];
                    if (ti >= 0) {
                        v.uv = uvs[static_cast<std::size_t>(ti)];
                    }
                    if (ni >= 0) {
                        v.normal = normals[static_cast<std::size_t>(ni)];
                        v.has_normal = length_squared(v.normal) > 0;
                    }
                    mesh.vertices.push_back(v);
                }
                face.push_back(it->second);
            }
            if (face.size() < 3) {
                lp.fail("a face needs at least three vertices");
            }
            for (std::size_t i = 1; i + 1 < face.size(); ++i) {
                mesh.indices.insert(mesh.indices.end(), {face[0], face[i], face[i + 1]});
            }
        }
    }
    return mesh;
}

ObjMesh load_obj_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        throw InputError("obj: cannot open " + path);
    }
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    try {
        return parse_obj(text);
    } catch (const InputError& e) {
        throw InputError(path + ": " + e.what());
    }
}

}  // namespace lumen
