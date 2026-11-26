#include "lumen/json.hpp"
#include "lumen/obj.hpp"
#include "testing.hpp"

using namespace lumen;

namespace {
std::string error_of(std::string_view text) {
    try {
        (void)parse_json(text);
    } catch (const InputError& e) {
        return e.what();
    }
    return "";
}
}  // namespace

TEST_CASE(json, parses_all_value_kinds) {
    const Json j = parse_json(R"({"a": [1, -2.5e2, true, false, null], "s": "x\ny", "o": {}, "e": []})");
    REQUIRE(j.is_object());
    const Json* a = j.find("a");
    REQUIRE(a != nullptr);
    CHECK_EQ(a->items().size(), 5U);
    CHECK_NEAR(a->items()[1].as_number(), -250.0, 0);
    CHECK(a->items()[2].as_bool());
    CHECK(!a->items()[3].as_bool());
    CHECK(a->items()[4].is_null());
    CHECK_EQ(j.find("s")->as_string(), std::string("x\ny"));
    CHECK(j.find("o")->is_object());
    CHECK(j.find("missing") == nullptr);
    CHECK_EQ(j.keys().front(), std::string("a"));  // document order is preserved
}

TEST_CASE(json, handles_unicode_escapes) {
    CHECK_EQ(parse_json(R"("\u00e9")").as_string(), std::string("\xc3\xa9"));
    CHECK_EQ(parse_json(R"("\u20ac")").as_string(), std::string("\xe2\x82\xac"));
    CHECK_EQ(parse_json(R"("\ud83d\ude00")").as_string(), std::string("\xf0\x9f\x98\x80"));  // surrogate pair
    CHECK_EQ(parse_json(R"("a\/b\\c\"")").as_string(), std::string("a/b\\c\""));
}

TEST_CASE(json, reports_line_and_column) {
    CHECK(error_of("{\n  \"a\": 1,\n  \"b\": ?\n}").find("line 3, column 8") != std::string::npos);
    CHECK(error_of("[1, 2").find("expected") != std::string::npos);
    CHECK(error_of("{\"a\":1,\"a\":2}").find("duplicate key") != std::string::npos);
}

TEST_CASE(json, rejects_malformed_documents) {
    for (const char* bad : {"", "   ", "{", "[1,]", "{\"a\":}", "01", "1.", ".5", "1e", "+1", "tru", "\"abc", "\"\\x\"",
                            "\"\\ud800\"", "\"\\udc00\"", "\"\\u12G4\"", "[1] 2", "nul", "{'a':1}", "\"\t\"", "1e999",
                            "{\"a\" 1}", "[,1]"}) {
        CHECK(!error_of(bad).empty());
    }
}

TEST_CASE(json, limits_nesting_depth) {
    std::string deep(200, '[');
    deep += std::string(200, ']');
    CHECK(error_of(deep).find("too deep") != std::string::npos);
    std::string ok(60, '[');
    ok += std::string(60, ']');
    CHECK(error_of(ok).empty());
}

TEST_CASE(json, dump_round_trips) {
    const char* doc = R"({"k":[1,2.5,-3e-07,"t\"q\n",true,null,{"x":[]}],"z":{}})";
    const Json j = parse_json(doc);
    const Json again = parse_json(j.dump());
    CHECK_EQ(again.dump(), j.dump());
    CHECK_NEAR(again.find("k")->items()[2].as_number(), -3e-7, 1e-20);
}

TEST_CASE(json, typed_accessors_throw_on_mismatch) {
    const Json j = parse_json("[1]");
    CHECK_THROWS(j.as_string());
    CHECK_THROWS(j.as_number());
    CHECK_THROWS(j.as_bool());
    CHECK_THROWS(j.keys());
    CHECK_THROWS(parse_json("1").items());
}

TEST_CASE(obj, parses_triangles_quads_and_negative_indices) {
    const char* text =
        "# a unit square as two quads sharing nothing\n"
        "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n"
        "vn 0 0 1\n"
        "f 1/1/1 2/2/1 3/3/1 4/4/1\n"
        "f -4 -3 -2\n";
    const ObjMesh m = parse_obj(text);
    CHECK_EQ(m.triangle_count(), 3U);  // quad -> 2 triangles, plus one
    REQUIRE(m.vertices.size() >= 4);
    CHECK(m.vertices[0].has_normal);
    CHECK_NEAR(m.vertices[2].uv.x, 1.0, 0);
    CHECK_EQ(m.indices[0], 0U);
    // Face "f -4 -3 -2" has no normal/uv so it creates distinct vertices.
    CHECK(!m.vertices.back().has_normal);
}

TEST_CASE(obj, shares_identical_vertices_and_ignores_unknown_statements) {
    const ObjMesh m = parse_obj("o thing\nusemtl x\ns off\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nf 1 2 3\nf 2 4 3\n");
    CHECK_EQ(m.vertices.size(), 4U);
    CHECK_EQ(m.triangle_count(), 2U);
}

TEST_CASE(obj, rejects_bad_input_with_line_numbers) {
    const auto message = [](const char* text) {
        try {
            (void)parse_obj(text);
        } catch (const InputError& e) {
            return std::string(e.what());
        }
        return std::string();
    };
    CHECK(message("v 0 0\n").find("line 1") != std::string::npos);
    CHECK(message("v 0 0 0\nf 1 2 3\n").find("line 2") != std::string::npos);  // index out of range
    CHECK(message("v 0 0 0\nv 1 0 0\nf 1 2\n").find("at least three") != std::string::npos);
    CHECK(message("v 0 0 0\nf 0 0 0\n").find("without a position") != std::string::npos);
    CHECK(message("v a b c\n").find("expected a number") != std::string::npos);
    CHECK(message("v 0 0 0\nf 1/x 1 1\n").find("bad face index") != std::string::npos);
    CHECK_THROWS(load_obj_file("/nonexistent/model.obj"));
}
