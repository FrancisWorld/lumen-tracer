// Strict RFC 8259 JSON parser used by the scene format. Syntax errors carry
// line and column; nesting depth and document size are bounded.
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "lumen/error.hpp"

namespace lumen {

class Json {
public:
    enum class Kind { Null, Bool, Number, String, Array, Object };

    Json() = default;
    static Json null() { return Json(); }
    static Json boolean(bool v);
    static Json number(double v);
    static Json string(std::string v);
    static Json array(std::vector<Json> items);
    static Json object(std::vector<std::string> keys, std::vector<Json> values);

    Kind kind() const { return kind_; }
    bool is_null() const { return kind_ == Kind::Null; }
    bool is_bool() const { return kind_ == Kind::Bool; }
    bool is_number() const { return kind_ == Kind::Number; }
    bool is_string() const { return kind_ == Kind::String; }
    bool is_array() const { return kind_ == Kind::Array; }
    bool is_object() const { return kind_ == Kind::Object; }

    /// Typed accessors throw InputError on a kind mismatch.
    bool as_bool() const;
    double as_number() const;
    const std::string& as_string() const;
    const std::vector<Json>& items() const;  // array elements
    const std::vector<std::string>& keys() const;  // object keys in document order
    const std::vector<Json>& values() const;  // object values, parallel to keys()

    /// Object lookup. Returns nullptr when the key is absent (or this is not an object).
    const Json* find(std::string_view key) const;

    /// Serialises compactly; round-trips through parse().
    std::string dump() const;

private:
    Kind kind_ = Kind::Null;
    bool bool_ = false;
    double number_ = 0;
    std::string string_;
    std::vector<Json> items_;  // array elements or object values
    std::vector<std::string> keys_;
};

/// Parses a complete document. Throws InputError("line L, column C: ...").
Json parse_json(std::string_view text);

const char* kind_name(Json::Kind kind);

}  // namespace lumen
