#include "lumen/deflate.hpp"

#include <algorithm>
#include <array>

namespace lumen {
namespace {

constexpr std::array<std::uint32_t, 256> make_crc_table() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t n = 0; n < 256; ++n) {
        std::uint32_t c = n;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1U) ? 0xedb88320U ^ (c >> 1U) : c >> 1U;
        }
        table[n] = c;
    }
    return table;
}
constexpr auto kCrcTable = make_crc_table();

constexpr std::array<std::uint16_t, 29> kLengthBase = {3,  4,  5,  6,  7,  8,  9,  10, 11,  13,  15,  17,  19,  23, 27,
                                                       31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr std::array<std::uint8_t, 29> kLengthExtra = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                                       2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr std::array<std::uint16_t, 30> kDistBase = {1,   2,   3,   4,   5,   7,    9,    13,   17,  25,
                                                     33,  49,  65,  97,  129, 193,  257,  385,  513, 769,
                                                     1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
constexpr std::array<std::uint8_t, 30> kDistExtra = {0, 0, 0, 0, 1, 1, 2, 2,  3,  3,  4,  4,  5,  5,  6,
                                                     6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

// ---------------------------------------------------------------- encoder

class BitWriter {
public:
    explicit BitWriter(Bytes& out) : out_(out) {}
    void put(std::uint32_t value, int bits) {  // LSB first
        acc_ |= static_cast<std::uint64_t>(value) << count_;
        count_ += bits;
        while (count_ >= 8) {
            out_.push_back(static_cast<std::uint8_t>(acc_ & 0xffU));
            acc_ >>= 8U;
            count_ -= 8;
        }
    }
    /// Huffman codes are packed most significant bit first.
    void put_code(std::uint32_t code, int bits) {
        std::uint32_t rev = 0;
        for (int i = 0; i < bits; ++i) {
            rev = (rev << 1U) | ((code >> i) & 1U);
        }
        put(rev, bits);
    }
    void flush() {
        if (count_ > 0) {
            out_.push_back(static_cast<std::uint8_t>(acc_ & 0xffU));
            acc_ = 0;
            count_ = 0;
        }
    }

private:
    Bytes& out_;
    std::uint64_t acc_ = 0;
    int count_ = 0;
};

void put_literal_length(BitWriter& w, unsigned sym) {
    if (sym < 144) {
        w.put_code(0x30U + sym, 8);
    } else if (sym < 256) {
        w.put_code(0x190U + (sym - 144), 9);
    } else if (sym < 280) {
        w.put_code(sym - 256, 7);
    } else {
        w.put_code(0xc0U + (sym - 280), 8);
    }
}

void put_match(BitWriter& w, unsigned length, unsigned distance) {
    unsigned li = 28;
    while (kLengthBase[li] > length) {
        --li;
    }
    put_literal_length(w, 257 + li);
    w.put(length - kLengthBase[li], kLengthExtra[li]);
    unsigned di = 29;
    while (kDistBase[di] > distance) {
        --di;
    }
    w.put_code(di, 5);
    w.put(distance - kDistBase[di], kDistExtra[di]);
}

Bytes deflate_fixed(std::span<const std::uint8_t> data) {
    constexpr std::size_t kWindow = 32768;
    constexpr unsigned kMinMatch = 3;
    constexpr unsigned kMaxMatch = 258;
    constexpr unsigned kMaxChain = 24;
    constexpr std::size_t kHashSize = 1U << 15U;

    Bytes out;
    out.reserve(data.size() / 2 + 64);
    BitWriter w(out);
    w.put(1, 1);  // final block
    w.put(1, 2);  // fixed Huffman

    std::vector<std::int64_t> head(kHashSize, -1);
    std::vector<std::int64_t> prev(data.size(), -1);
    auto hash_at = [&](std::size_t i) {
        const std::uint32_t v = data[i] | (static_cast<std::uint32_t>(data[i + 1]) << 8U) |
                                (static_cast<std::uint32_t>(data[i + 2]) << 16U);
        return (v * 2654435761U) >> 17U;  // 15 bits
    };
    auto insert = [&](std::size_t i) {
        if (i + kMinMatch <= data.size()) {
            const auto h = hash_at(i);
            prev[i] = head[h];
            head[h] = static_cast<std::int64_t>(i);
        }
    };

    std::size_t i = 0;
    while (i < data.size()) {
        unsigned best_len = 0;
        unsigned best_dist = 0;
        if (i + kMinMatch <= data.size()) {
            std::int64_t cand = head[hash_at(i)];
            const std::size_t limit = std::min<std::size_t>(kMaxMatch, data.size() - i);
            for (unsigned chain = 0; cand >= 0 && chain < kMaxChain; ++chain) {
                const auto c = static_cast<std::size_t>(cand);
                if (i - c > kWindow) {
                    break;
                }
                unsigned len = 0;
                while (len < limit && data[c + len] == data[i + len]) {
                    ++len;
                }
                if (len > best_len) {
                    best_len = len;
                    best_dist = static_cast<unsigned>(i - c);
                    if (len == limit) {
                        break;
                    }
                }
                cand = prev[c];
            }
        }
        if (best_len >= kMinMatch) {
            put_match(w, best_len, best_dist);
            for (unsigned k = 0; k < best_len; ++k) {
                insert(i + k);
            }
            i += best_len;
        } else {
            put_literal_length(w, data[i]);
            insert(i);
            ++i;
        }
    }
    put_literal_length(w, 256);
    w.flush();
    return out;
}

// ---------------------------------------------------------------- decoder

class BitReader {
public:
    explicit BitReader(std::span<const std::uint8_t> in) : in_(in) {}
    bool ok() const { return !overrun_; }
    std::uint32_t bits(int n) {
        while (count_ < n) {
            if (pos_ >= in_.size()) {
                overrun_ = true;
                return 0;
            }
            acc_ |= static_cast<std::uint32_t>(in_[pos_++]) << count_;
            count_ += 8;
        }
        const std::uint32_t v = acc_ & ((1U << n) - 1U);
        acc_ >>= n;
        count_ -= n;
        return v;
    }
    void align() {
        acc_ = 0;
        count_ = 0;
    }
    std::span<const std::uint8_t> take_bytes(std::size_t n) {
        if (pos_ + n > in_.size()) {
            overrun_ = true;
            return {};
        }
        const auto s = in_.subspan(pos_, n);
        pos_ += n;
        return s;
    }

private:
    std::span<const std::uint8_t> in_;
    std::size_t pos_ = 0;
    std::uint32_t acc_ = 0;
    int count_ = 0;
    bool overrun_ = false;
};

/// Canonical Huffman decoder (counts per length + symbols sorted by code).
struct Huffman {
    std::array<std::uint16_t, 16> count{};
    std::array<std::uint16_t, 288> symbol{};

    /// Returns false for an over-subscribed code. Incomplete codes are legal for single-symbol distance trees.
    bool build(std::span<const std::uint8_t> lengths) {
        count.fill(0);
        for (const auto l : lengths) {
            ++count[l];
        }
        count[0] = 0;
        int left = 1;
        for (int len = 1; len < 16; ++len) {
            left <<= 1;
            left -= count[len];
            if (left < 0) {
                return false;
            }
        }
        std::array<std::uint16_t, 16> offs{};
        for (int len = 1; len < 15; ++len) {
            offs[len + 1] = static_cast<std::uint16_t>(offs[len] + count[len]);
        }
        for (std::size_t s = 0; s < lengths.size(); ++s) {
            if (lengths[s] != 0) {
                symbol[offs[lengths[s]]++] = static_cast<std::uint16_t>(s);
            }
        }
        return true;
    }

    int decode(BitReader& r) const {
        int code = 0;
        int first = 0;
        int index = 0;
        for (int len = 1; len < 16; ++len) {
            code |= static_cast<int>(r.bits(1));
            const int c = count[len];
            if (code - c < first) {
                return symbol[index + (code - first)];
            }
            index += c;
            first += c;
            first <<= 1;
            code <<= 1;
            if (!r.ok()) {
                return -1;
            }
        }
        return -1;
    }
};

using InflateResult = std::expected<void, std::string>;

InflateResult inflate_block(BitReader& r, const Huffman& lit, const Huffman& dist, Bytes& out, std::size_t max_output) {
    for (;;) {
        const int sym = lit.decode(r);
        if (sym < 0 || !r.ok()) {
            return std::unexpected("invalid Huffman code or truncated stream");
        }
        if (sym < 256) {
            if (out.size() >= max_output) {
                return std::unexpected("decoded data exceeds the size limit");
            }
            out.push_back(static_cast<std::uint8_t>(sym));
        } else if (sym == 256) {
            return {};
        } else {
            const auto li = static_cast<std::size_t>(sym - 257);
            if (li >= kLengthBase.size()) {
                return std::unexpected("invalid length symbol");
            }
            const std::size_t length = kLengthBase[li] + r.bits(kLengthExtra[li]);
            const int ds = dist.decode(r);
            if (ds < 0 || ds >= 30 || !r.ok()) {
                return std::unexpected("invalid distance symbol");
            }
            const std::size_t distance = kDistBase[static_cast<std::size_t>(ds)] + r.bits(kDistExtra[static_cast<std::size_t>(ds)]);
            if (distance > out.size()) {
                return std::unexpected("distance reaches before the start of the output");
            }
            if (out.size() + length > max_output) {
                return std::unexpected("decoded data exceeds the size limit");
            }
            const std::size_t from = out.size() - distance;
            for (std::size_t k = 0; k < length; ++k) {
                out.push_back(out[from + k]);  // may overlap itself, byte by byte on purpose
            }
        }
    }
}

InflateResult read_dynamic_tables(BitReader& r, Huffman& lit, Huffman& dist) {
    const unsigned nlen = r.bits(5) + 257;
    const unsigned ndist = r.bits(5) + 1;
    const unsigned ncode = r.bits(4) + 4;
    if (nlen > 286 || ndist > 30) {
        return std::unexpected("too many length or distance codes");
    }
    static constexpr std::array<std::uint8_t, 19> order = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    std::array<std::uint8_t, 19> cl{};
    for (unsigned i = 0; i < ncode; ++i) {
        cl[order[i]] = static_cast<std::uint8_t>(r.bits(3));
    }
    Huffman clh;
    if (!clh.build(cl)) {
        return std::unexpected("bad code-length code");
    }
    std::array<std::uint8_t, 320> lengths{};
    unsigned idx = 0;
    while (idx < nlen + ndist) {
        const int sym = clh.decode(r);
        if (sym < 0 || !r.ok()) {
            return std::unexpected("bad code-length symbol");
        }
        if (sym < 16) {
            lengths[idx++] = static_cast<std::uint8_t>(sym);
            continue;
        }
        std::uint8_t value = 0;
        unsigned rep = 0;
        if (sym == 16) {
            if (idx == 0) {
                return std::unexpected("repeat with no previous length");
            }
            value = lengths[idx - 1];
            rep = 3 + r.bits(2);
        } else if (sym == 17) {
            rep = 3 + r.bits(3);
        } else {
            rep = 11 + r.bits(7);
        }
        if (idx + rep > nlen + ndist) {
            return std::unexpected("code lengths overflow");
        }
        while (rep-- > 0) {
            lengths[idx++] = value;
        }
    }
    if (lengths[256] == 0) {
        return std::unexpected("missing end-of-block code");
    }
    if (!lit.build(std::span(lengths).first(nlen)) || !dist.build(std::span(lengths).subspan(nlen, ndist))) {
        return std::unexpected("over-subscribed Huffman code");
    }
    return {};
}

}  // namespace

std::uint32_t crc32(std::span<const std::uint8_t> data, std::uint32_t seed) {
    std::uint32_t c = ~seed;
    for (const auto b : data) {
        c = kCrcTable[(c ^ b) & 0xffU] ^ (c >> 8U);
    }
    return ~c;
}

std::uint32_t adler32(std::span<const std::uint8_t> data) {
    constexpr std::uint32_t kMod = 65521;
    std::uint32_t a = 1;
    std::uint32_t b = 0;
    for (const auto byte : data) {
        a = (a + byte) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16U) | a;
}

Bytes zlib_compress(std::span<const std::uint8_t> data) {
    Bytes out = {0x78, 0x01};  // deflate, 32K window, no preset dictionary, check bits valid
    const Bytes body = deflate_fixed(data);
    out.insert(out.end(), body.begin(), body.end());
    const std::uint32_t a = adler32(data);
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>((a >> shift) & 0xffU));
    }
    return out;
}

std::expected<Bytes, std::string> zlib_decompress(std::span<const std::uint8_t> data, std::size_t max_output) {
    if (data.size() < 6) {
        return std::unexpected("zlib stream too short");
    }
    if ((data[0] & 0x0fU) != 8 || ((data[0] << 8U) | data[1]) % 31 != 0) {
        return std::unexpected("bad zlib header");
    }
    if ((data[1] & 0x20U) != 0) {
        return std::unexpected("preset dictionaries are not supported");
    }
    BitReader r(data.subspan(2, data.size() - 6));
    Bytes out;
    bool final_block = false;
    while (!final_block) {
        final_block = r.bits(1) != 0;
        const std::uint32_t type = r.bits(2);
        if (!r.ok()) {
            return std::unexpected("truncated deflate stream");
        }
        if (type == 0) {
            r.align();
            const auto hdr = r.take_bytes(4);
            if (hdr.size() != 4) {
                return std::unexpected("truncated stored block");
            }
            const std::size_t len = hdr[0] | (hdr[1] << 8U);
            const std::size_t nlen = hdr[2] | (hdr[3] << 8U);
            if ((len ^ 0xffffU) != nlen) {
                return std::unexpected("stored block length check failed");
            }
            const auto body = r.take_bytes(len);
            if (body.size() != len) {
                return std::unexpected("truncated stored block");
            }
            if (out.size() + len > max_output) {
                return std::unexpected("decoded data exceeds the size limit");
            }
            out.insert(out.end(), body.begin(), body.end());
        } else if (type == 1 || type == 2) {
            Huffman lit;
            Huffman dist;
            if (type == 1) {
                std::array<std::uint8_t, 288> l{};
                std::fill(l.begin(), l.begin() + 144, std::uint8_t{8});
                std::fill(l.begin() + 144, l.begin() + 256, std::uint8_t{9});
                std::fill(l.begin() + 256, l.begin() + 280, std::uint8_t{7});
                std::fill(l.begin() + 280, l.end(), std::uint8_t{8});
                lit.build(l);
                std::array<std::uint8_t, 30> d{};
                d.fill(5);
                dist.build(d);
            } else if (auto t = read_dynamic_tables(r, lit, dist); !t) {
                return std::unexpected(t.error());
            }
            if (auto b = inflate_block(r, lit, dist, out, max_output); !b) {
                return std::unexpected(b.error());
            }
        } else {
            return std::unexpected("invalid block type");
        }
    }
    const std::uint32_t expected = (static_cast<std::uint32_t>(data[data.size() - 4]) << 24U) |
                                   (static_cast<std::uint32_t>(data[data.size() - 3]) << 16U) |
                                   (static_cast<std::uint32_t>(data[data.size() - 2]) << 8U) | data[data.size() - 1];
    if (adler32(out) != expected) {
        return std::unexpected("Adler-32 checksum mismatch");
    }
    return out;
}

}  // namespace lumen
