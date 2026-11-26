#include "lumen/png.hpp"

#include "lumen/error.hpp"
#include "lumen/fileio.hpp"

#include <array>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace lumen {
namespace {

constexpr std::array<std::uint8_t, 8> kSignature = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
constexpr std::size_t kMaxDecodedBytes = std::size_t{1} << 28;  // 256 MiB

void put_u32(Bytes& out, std::uint32_t v) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>((v >> shift) & 0xffU));
    }
}

void put_chunk(Bytes& out, const char (&type)[5], std::span<const std::uint8_t> data) {
    put_u32(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    put_u32(out, crc32(std::span(out).subspan(start)));
}

std::uint8_t paeth(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a);
    const int pb = std::abs(p - b);
    const int pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) {
        return static_cast<std::uint8_t>(a);
    }
    return static_cast<std::uint8_t>(pb <= pc ? b : c);
}

/// Apply filter `type` to one row. `prev` is the previous raw row (zeros for the first).
void filter_row(int type, std::span<const std::uint8_t> row, std::span<const std::uint8_t> prev, std::size_t bpp,
                std::uint8_t* out) {
    for (std::size_t i = 0; i < row.size(); ++i) {
        const int a = i >= bpp ? row[i - bpp] : 0;
        const int b = prev[i];
        const int c = i >= bpp ? prev[i - bpp] : 0;
        int predictor = 0;
        switch (type) {
            case 1: predictor = a; break;
            case 2: predictor = b; break;
            case 3: predictor = (a + b) / 2; break;
            case 4: predictor = paeth(a, b, c); break;
            default: break;
        }
        out[i] = static_cast<std::uint8_t>(row[i] - predictor);
    }
}

}  // namespace

Bytes encode_png(const Image8& image) {
    if (image.channels < 1 || image.channels > 4 || image.width == 0 || image.height == 0) {
        throw std::invalid_argument("png: unsupported image shape");
    }
    const std::size_t stride = static_cast<std::size_t>(image.width) * image.channels;
    if (image.pixels.size() != stride * image.height) {
        throw std::invalid_argument("png: pixel buffer size does not match dimensions");
    }
    // Filter each row with the type that minimises the sum of absolute residuals.
    Bytes filtered;
    filtered.reserve((stride + 1) * image.height);
    const Bytes zero_row(stride, 0);
    Bytes candidate(stride);
    Bytes best(stride);
    for (std::uint32_t y = 0; y < image.height; ++y) {
        const std::span<const std::uint8_t> row(image.pixels.data() + y * stride, stride);
        const std::span<const std::uint8_t> prev =
            y == 0 ? std::span<const std::uint8_t>(zero_row) : std::span<const std::uint8_t>(image.pixels.data() + (y - 1) * stride, stride);
        int best_type = 0;
        std::uint64_t best_score = ~std::uint64_t{0};
        for (int type = 0; type <= 4; ++type) {
            filter_row(type, row, prev, image.channels, candidate.data());
            std::uint64_t score = 0;
            for (const auto v : candidate) {
                score += v < 128 ? v : 256U - v;
            }
            if (score < best_score) {
                best_score = score;
                best_type = type;
                best = candidate;
            }
        }
        filtered.push_back(static_cast<std::uint8_t>(best_type));
        filtered.insert(filtered.end(), best.begin(), best.end());
    }

    static constexpr std::array<std::uint8_t, 5> kColorType = {0, 0, 4, 2, 6};
    Bytes out(kSignature.begin(), kSignature.end());
    Bytes ihdr;
    put_u32(ihdr, image.width);
    put_u32(ihdr, image.height);
    ihdr.insert(ihdr.end(), {8, kColorType[image.channels], 0, 0, 0});
    put_chunk(out, "IHDR", ihdr);
    put_chunk(out, "IDAT", zlib_compress(filtered));
    put_chunk(out, "IEND", {});
    return out;
}

std::expected<Image8, std::string> decode_png(std::span<const std::uint8_t> data) {
    if (data.size() < kSignature.size() || std::memcmp(data.data(), kSignature.data(), kSignature.size()) != 0) {
        return std::unexpected("not a PNG file");
    }
    std::size_t pos = kSignature.size();
    Image8 image;
    Bytes idat;
    bool have_header = false;
    bool have_end = false;
    auto u32 = [&](std::size_t at) {
        return (static_cast<std::uint32_t>(data[at]) << 24U) | (static_cast<std::uint32_t>(data[at + 1]) << 16U) |
               (static_cast<std::uint32_t>(data[at + 2]) << 8U) | data[at + 3];
    };
    while (pos + 12 <= data.size() && !have_end) {
        const std::uint32_t len = u32(pos);
        if (len > data.size() - pos - 12) {
            return std::unexpected("truncated chunk");
        }
        const auto type = data.subspan(pos + 4, 4);
        const auto body = data.subspan(pos + 8, len);
        if (crc32(data.subspan(pos + 4, 4 + static_cast<std::size_t>(len))) != u32(pos + 8 + len)) {
            return std::unexpected("chunk CRC mismatch");
        }
        auto is = [&](const char* name) { return std::memcmp(type.data(), name, 4) == 0; };
        if (is("IHDR")) {
            if (len != 13) {
                return std::unexpected("bad IHDR");
            }
            image.width = u32(pos + 8);
            image.height = u32(pos + 12);
            const std::uint8_t depth = body[8];
            const std::uint8_t color = body[9];
            if (depth != 8) {
                return std::unexpected("only 8-bit PNGs are supported");
            }
            if (body[12] != 0) {
                return std::unexpected("interlaced PNGs are not supported");
            }
            switch (color) {
                case 0: image.channels = 1; break;
                case 2: image.channels = 3; break;
                case 4: image.channels = 2; break;
                case 6: image.channels = 4; break;
                default: return std::unexpected("palette PNGs are not supported");
            }
            if (image.width == 0 || image.height == 0 || image.width > 65536 || image.height > 65536) {
                return std::unexpected("unreasonable image dimensions");
            }
            have_header = true;
        } else if (is("IDAT")) {
            idat.insert(idat.end(), body.begin(), body.end());
        } else if (is("IEND")) {
            have_end = true;
        } else if ((type[0] & 0x20U) == 0) {
            return std::unexpected("unknown critical chunk");
        }
        pos += 12 + len;
    }
    if (!have_header || !have_end) {
        return std::unexpected("missing IHDR or IEND");
    }
    const std::size_t stride = static_cast<std::size_t>(image.width) * image.channels;
    const std::size_t expected = (stride + 1) * image.height;
    if (expected > kMaxDecodedBytes) {
        return std::unexpected("image too large");
    }
    auto raw = zlib_decompress(idat, expected);
    if (!raw) {
        return std::unexpected("IDAT: " + raw.error());
    }
    if (raw->size() != expected) {
        return std::unexpected("IDAT size does not match the image dimensions");
    }
    image.pixels.assign(stride * image.height, 0);
    const std::size_t bpp = image.channels;
    for (std::uint32_t y = 0; y < image.height; ++y) {
        const std::uint8_t type = (*raw)[y * (stride + 1)];
        const std::uint8_t* src = raw->data() + y * (stride + 1) + 1;
        std::uint8_t* dst = image.pixels.data() + y * stride;
        const std::uint8_t* up = y > 0 ? dst - stride : nullptr;
        if (type > 4) {
            return std::unexpected("invalid filter type");
        }
        for (std::size_t i = 0; i < stride; ++i) {
            const int a = i >= bpp ? dst[i - bpp] : 0;
            const int b = up ? up[i] : 0;
            const int c = (up && i >= bpp) ? up[i - bpp] : 0;
            int predictor = 0;
            switch (type) {
                case 1: predictor = a; break;
                case 2: predictor = b; break;
                case 3: predictor = (a + b) / 2; break;
                case 4: predictor = paeth(a, b, c); break;
                default: break;
            }
            dst[i] = static_cast<std::uint8_t>(src[i] + predictor);
        }
    }
    return image;
}

void write_png_file(const std::string& path, const Image8& image) {
    const Bytes bytes = encode_png(image);
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) {
        throw std::runtime_error("cannot open for writing: " + path);
    }
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!f) {
        throw std::runtime_error("write failed: " + path);
    }
}

std::expected<Image8, std::string> read_png_file(const std::string& path) {
    try {
        const std::string text = read_file_bounded(path, kMaxAssetBytes);
        return decode_png(std::span(reinterpret_cast<const std::uint8_t*>(text.data()), text.size()));
    } catch (const InputError& e) {
        return std::unexpected(std::string(e.what()));
    }
}

}  // namespace lumen
