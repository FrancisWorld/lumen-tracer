#include "lumen/json.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>

namespace lumen {
namespace {

constexpr int kMaxDepth = 64;
constexpr std::size_t kMaxDocument = std::size_t{64} << 20;

class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}

    Json parse_document() {
        if (text_.size() > kMaxDocument) {
            fail("document too large");
        }
        skip_ws();
        Json v = parse_value(0);
        skip_ws();
        if (pos_ != text_.size()) {
            fail("unexpected trailing characters");
        }
        return v;
    }

private:
    [[noreturn]] void fail(const std::string& what) const {
        std::size_t line = 1;
        std::size_t col = 1;
        for (std::size_t i = 0; i < pos_ && i < text_.size(); ++i) {
            if (text_[i] == '\n') {
                ++line;
                col = 1;
            } else {
                ++col;
            }
        }
        throw InputError("json: line " + std::to_string(line) + ", column " + std::to_string(col) + ": " + what);
    }

    bool at_end() const { return pos_ >= text_.size(); }
    char peek() const { return at_end() ? '\0' : text_[pos_]; }

    void skip_ws() {
        while (!at_end() && (text_[pos_] == ' ' || text_[pos_] == '\t' || text_[pos_] == '\n' || text_[pos_] == '\r')) {
            ++pos_;
        }
    }

    void expect(char c) {
        if (peek() != c) {
            fail(std::string("expected '") + c + "'");
        }
        ++pos_;
    }

    void expect_word(std::string_view word) {
        if (text_.substr(pos_, word.size()) != word) {
            fail("invalid literal");
        }
        pos_ += word.size();
    }

    Json parse_value(int depth) {
        if (depth > kMaxDepth) {
            fail("nesting too deep");
        }
        if (at_end()) {
            fail("unexpected end of input");
        }
        switch (peek()) {
            case '{': return parse_object(depth);
            case '[': return parse_array(depth);
            case '"': return Json::string(parse_string());
            case 't': expect_word("true"); return Json::boolean(true);
            case 'f': expect_word("false"); return Json::boolean(false);
            case 'n': expect_word("null"); return Json::null();
            default: return parse_number();
        }
    }

    Json parse_object(int depth) {
        expect('{');
        std::vector<std::string> keys;
        std::vector<Json> values;
        skip_ws();
        if (peek() == '}') {
            ++pos_;
            return Json::object({}, {});
        }
        for (;;) {
            skip_ws();
            if (peek() != '"') {
                fail("expected a string key");
            }
            std::string key = parse_string();
            for (const auto& existing : keys) {
                if (existing == key) {
                    fail("duplicate key \"" + key + "\"");
                }
            }
            skip_ws();
            expect(':');
            skip_ws();
            values.push_back(parse_value(depth + 1));
            keys.push_back(std::move(key));
            skip_ws();
            if (peek() == ',') {
                ++pos_;
                continue;
            }
            expect('}');
            return Json::object(std::move(keys), std::move(values));
        }
    }

    Json parse_array(int depth) {
        expect('[');
        std::vector<Json> items;
        skip_ws();
        if (peek() == ']') {
            ++pos_;
            return Json::array({});
        }
        for (;;) {
            skip_ws();
            items.push_back(parse_value(depth + 1));
            skip_ws();
            if (peek() == ',') {
                ++pos_;
                continue;
            }
            expect(']');
            return Json::array(std::move(items));
        }
    }

    unsigned parse_hex4() {
        if (pos_ + 4 > text_.size()) {
            fail("truncated \\u escape");
        }
        unsigned v = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = text_[pos_++];
            v <<= 4U;
            if (c >= '0' && c <= '9') {
                v |= static_cast<unsigned>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                v |= static_cast<unsigned>(c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                v |= static_cast<unsigned>(c - 'A' + 10);
            } else {
                --pos_;
                fail("invalid hex digit in \\u escape");
            }
        }
        return v;
    }

    static void append_utf8(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xc0 | (cp >> 6U));
            out += static_cast<char>(0x80 | (cp & 0x3fU));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xe0 | (cp >> 12U));
            out += static_cast<char>(0x80 | ((cp >> 6U) & 0x3fU));
            out += static_cast<char>(0x80 | (cp & 0x3fU));
        } else {
            out += static_cast<char>(0xf0 | (cp >> 18U));
            out += static_cast<char>(0x80 | ((cp >> 12U) & 0x3fU));
            out += static_cast<char>(0x80 | ((cp >> 6U) & 0x3fU));
            out += static_cast<char>(0x80 | (cp & 0x3fU));
        }
    }

    /// Copies one well-formed UTF-8 sequence (RFC 3629) starting at pos_, or fails. Rejects stray
    /// continuation bytes, overlong forms, UTF-16 surrogates and code points above U+10FFFF.
    void copy_utf8_sequence(std::string& out) {
        const auto lead = static_cast<unsigned char>(text_[pos_]);
        std::size_t length = 0;
        unsigned char lo = 0x80;  // allowed range of the first continuation byte
        unsigned char hi = 0xbf;
        if (lead >= 0xc2 && lead <= 0xdf) {
            length = 2;
        } else if (lead >= 0xe0 && lead <= 0xef) {
            length = 3;
            if (lead == 0xe0) lo = 0xa0;  // excludes overlong 3-byte forms
            if (lead == 0xed) hi = 0x9f;  // excludes surrogates U+D800..U+DFFF
        } else if (lead >= 0xf0 && lead <= 0xf4) {
            length = 4;
            if (lead == 0xf0) lo = 0x90;  // excludes overlong 4-byte forms
            if (lead == 0xf4) hi = 0x8f;  // excludes code points above U+10FFFF
        } else {
            fail("invalid UTF-8 in string");
        }
        if (pos_ + length > text_.size()) {
            fail("truncated UTF-8 sequence in string");
        }
        for (std::size_t i = 1; i < length; ++i) {
            const auto b = static_cast<unsigned char>(text_[pos_ + i]);
            const unsigned char min = i == 1 ? lo : 0x80;
            const unsigned char max = i == 1 ? hi : 0xbf;
            if (b < min || b > max) {
                pos_ += i;
                fail("invalid UTF-8 in string");
            }
        }
        out.append(text_.substr(pos_, length));
        pos_ += length;
    }

    std::string parse_string() {
        expect('"');
        std::string out;
        for (;;) {
            if (at_end()) {
                fail("unterminated string");
            }
            const char c = text_[pos_++];
            if (c == '"') {
                return out;
            }
            if (static_cast<unsigned char>(c) < 0x20) {
                --pos_;
                fail("control character in string");
            }
            if (c != '\\') {
                if (static_cast<unsigned char>(c) >= 0x80) {
                    --pos_;
                    copy_utf8_sequence(out);
                } else {
                    out += c;
                }
                continue;
            }
            if (at_end()) {
                fail("unterminated escape");
            }
            const char e = text_[pos_++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp = parse_hex4();
                    if (cp >= 0xd800 && cp <= 0xdbff) {
                        if (text_.substr(pos_, 2) != "\\u") {
                            fail("unpaired surrogate");
                        }
                        pos_ += 2;
                        const unsigned low = parse_hex4();
                        if (low < 0xdc00 || low > 0xdfff) {
                            fail("invalid low surrogate");
                        }
                        cp = 0x10000 + ((cp - 0xd800) << 10U) + (low - 0xdc00);
                    } else if (cp >= 0xdc00 && cp <= 0xdfff) {
                        fail("unpaired surrogate");
                    }
                    append_utf8(out, cp);
                    break;
                }
                default: --pos_; fail("invalid escape sequence");
            }
        }
    }

    Json parse_number() {
        const std::size_t start = pos_;
        auto digits = [&] {
            std::size_t n = 0;
            while (peek() >= '0' && peek() <= '9') {
                ++pos_;
                ++n;
            }
            return n;
        };
        if (peek() == '-') {
            ++pos_;
        }
        if (peek() == '0') {
            ++pos_;
        } else if (digits() == 0) {
            pos_ = start;
            fail("invalid value");
        }
        if (peek() == '.') {
            ++pos_;
            if (digits() == 0) {
                fail("digits expected after decimal point");
            }
        }
        if (peek() == 'e' || peek() == 'E') {
            ++pos_;
            if (peek() == '+' || peek() == '-') {
                ++pos_;
            }
            if (digits() == 0) {
                fail("digits expected in exponent");
            }
        }
        double v = 0;
        const auto [ptr, ec] = std::from_chars(text_.data() + start, text_.data() + pos_, v);
        if (ec != std::errc() || ptr != text_.data() + pos_ || !std::isfinite(v)) {
            pos_ = start;
            fail("number out of range");
        }
        return Json::number(v);
    }

    std::string_view text_;
    std::size_t pos_ = 0;
};

void dump_string(std::string& out, const std::string& s) {
    out += '"';
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

void dump_into(std::string& out, const Json& j) {
    switch (j.kind()) {
        case Json::Kind::Null: out += "null"; break;
        case Json::Kind::Bool: out += j.as_bool() ? "true" : "false"; break;
        case Json::Kind::Number: {
            char buf[40];
            const auto r = std::to_chars(buf, buf + sizeof buf, j.as_number());
            out.append(buf, r.ptr);
            break;
        }
        case Json::Kind::String: dump_string(out, j.as_string()); break;
        case Json::Kind::Array: {
            out += '[';
            bool first = true;
            for (const Json& v : j.items()) {
                if (!first) {
                    out += ',';
                }
                first = false;
                dump_into(out, v);
            }
            out += ']';
            break;
        }
        case Json::Kind::Object: {
            out += '{';
            for (std::size_t i = 0; i < j.keys().size(); ++i) {
                if (i != 0) {
                    out += ',';
                }
                dump_string(out, j.keys()[i]);
                out += ':';
                dump_into(out, j.values()[i]);
            }
            out += '}';
            break;
        }
    }
}

}  // namespace

const char* kind_name(Json::Kind kind) {
    switch (kind) {
        case Json::Kind::Null: return "null";
        case Json::Kind::Bool: return "boolean";
        case Json::Kind::Number: return "number";
        case Json::Kind::String: return "string";
        case Json::Kind::Array: return "array";
        case Json::Kind::Object: return "object";
    }
    return "unknown";
}

Json Json::boolean(bool v) {
    Json j;
    j.kind_ = Kind::Bool;
    j.bool_ = v;
    return j;
}
Json Json::number(double v) {
    Json j;
    j.kind_ = Kind::Number;
    j.number_ = v;
    return j;
}
Json Json::string(std::string v) {
    Json j;
    j.kind_ = Kind::String;
    j.string_ = std::move(v);
    return j;
}
Json Json::array(std::vector<Json> items) {
    Json j;
    j.kind_ = Kind::Array;
    j.items_ = std::move(items);
    return j;
}
Json Json::object(std::vector<std::string> keys, std::vector<Json> values) {
    Json j;
    j.kind_ = Kind::Object;
    j.keys_ = std::move(keys);
    j.items_ = std::move(values);
    return j;
}

namespace {
[[noreturn]] void type_error(Json::Kind want, Json::Kind got) {
    throw InputError(std::string("expected ") + kind_name(want) + " but found " + kind_name(got));
}
}  // namespace

bool Json::as_bool() const {
    if (kind_ != Kind::Bool) type_error(Kind::Bool, kind_);
    return bool_;
}
double Json::as_number() const {
    if (kind_ != Kind::Number) type_error(Kind::Number, kind_);
    return number_;
}
const std::string& Json::as_string() const {
    if (kind_ != Kind::String) type_error(Kind::String, kind_);
    return string_;
}
const std::vector<Json>& Json::items() const {
    if (kind_ != Kind::Array) type_error(Kind::Array, kind_);
    return items_;
}
const std::vector<std::string>& Json::keys() const {
    if (kind_ != Kind::Object) type_error(Kind::Object, kind_);
    return keys_;
}
const std::vector<Json>& Json::values() const {
    if (kind_ != Kind::Object) type_error(Kind::Object, kind_);
    return items_;
}

const Json* Json::find(std::string_view key) const {
    if (kind_ != Kind::Object) {
        return nullptr;
    }
    for (std::size_t i = 0; i < keys_.size(); ++i) {
        if (keys_[i] == key) {
            return &items_[i];
        }
    }
    return nullptr;
}

std::string Json::dump() const {
    std::string out;
    dump_into(out, *this);
    return out;
}

Json parse_json(std::string_view text) { return Parser(text).parse_document(); }

}  // namespace lumen
