#include <cstdio>
#include <filesystem>
#include <string>

#include "lumen/deflate.hpp"
#include "lumen/png.hpp"
#include "lumen/rng.hpp"
#include "testing.hpp"

using namespace lumen;

namespace {

Bytes bytes_of(const std::string& s) { return Bytes(s.begin(), s.end()); }

Bytes random_bytes(std::size_t n, std::uint64_t seed, std::uint32_t alphabet) {
    Pcg32 rng(seed);
    Bytes b(n);
    for (auto& v : b) {
        v = static_cast<std::uint8_t>(rng.next_u32() % alphabet);
    }
    return b;
}

}  // namespace

TEST_CASE(deflate, checksums_match_reference_values) {
    const Bytes check = bytes_of("123456789");
    CHECK_EQ(crc32(check), 0xcbf43926U);       // standard CRC-32 check value
    CHECK_EQ(adler32(bytes_of("Wikipedia")), 0x11e60398U);
    CHECK_EQ(crc32({}), 0U);
    CHECK_EQ(adler32({}), 1U);
}

TEST_CASE(deflate, round_trips_many_shapes) {
    const std::size_t sizes[] = {0, 1, 2, 3, 100, 257, 1000, 70000};
    for (const std::size_t n : sizes) {
        for (const std::uint32_t alphabet : {1U, 4U, 256U}) {
            const Bytes data = random_bytes(n, n + alphabet, alphabet);
            const Bytes z = zlib_compress(data);
            const auto back = zlib_decompress(z, 1 << 20);
            REQUIRE(back.has_value());
            CHECK(*back == data);
        }
    }
}

TEST_CASE(deflate, compresses_repetitive_data) {
    const Bytes data(100000, 7);
    CHECK(zlib_compress(data).size() < 1000);
}

TEST_CASE(deflate, inflates_dynamic_huffman_stream_made_by_zlib) {
    // Produced by Python's zlib at level 9: block type 2 (dynamic Huffman).
    const Bytes z = {120, 218, 221, 141, 217, 17,  194, 64,  12,  67,  91,  217, 10,  232, 201, 58,  184, 97,  33,  36,
                     92,  213, 163, 77,  25,  124, 216, 51,  122, 214, 27,  207, 123, 183, 251, 114, 224, 169, 97,  234,
                     175, 107, 219, 246, 119, 59,  46,  151, 219, 163, 245, 167, 167, 54,  231, 124, 174, 239, 167, 169,
                     239, 54,  107, 250, 235, 50,  108, 208, 114, 185, 68,  3,   178, 37,  0,   86,  21,  92,  44,  42,
                     32,  3,   6,   100, 107, 0,   51,  181, 136, 197, 97,  219, 169, 50,  119, 41,  130, 16,  39,  118,
                     232, 232, 165, 17,  184, 70,  228, 3,   147, 72,  232, 7,   194, 17,  144, 36};
    std::string expected;
    for (int i = 0; i < 6; ++i) {
        expected += "the quick brown fox jumps over the lazy dog. ";
    }
    expected +=
        "beebcedeaeadcebbdeeddbbbedaabeacacdedddedbcaabdbcdcdedeceedebcacebceeeabeccaaddacadbacddaaeeadececebacaaaeeabdcecbacccbd";
    const auto out = zlib_decompress(z, 4096);
    REQUIRE(out.has_value());
    CHECK(*out == bytes_of(expected));
}

TEST_CASE(deflate, rejects_corrupt_and_hostile_streams) {
    const Bytes data = random_bytes(5000, 1, 8);
    Bytes z = zlib_compress(data);

    Bytes bad_sum = z;
    bad_sum.back() ^= 0xff;
    CHECK(!zlib_decompress(bad_sum, 1 << 20));

    Bytes bad_header = z;
    bad_header[0] = 0x79;
    CHECK(!zlib_decompress(bad_header, 1 << 20));

    CHECK(!zlib_decompress(std::span(z).first(z.size() / 2), 1 << 20));  // truncated
    CHECK(!zlib_decompress({}, 1 << 20));
    CHECK(!zlib_decompress(z, 100));  // output limit enforced (decompression bomb guard)

    // Reserved block type 3.
    const Bytes reserved = {0x78, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01};
    CHECK(!zlib_decompress(reserved, 100));

    // Every single-byte corruption must yield an error or a checksum failure, never a crash.
    for (std::size_t i = 2; i < z.size(); i += 7) {
        Bytes c = z;
        c[i] ^= 0x5a;
        const auto r = zlib_decompress(c, 1 << 16);
        CHECK(!r.has_value() || *r != data || c == z);
    }
}

TEST_CASE(png, round_trip_for_every_channel_count) {
    for (std::uint32_t channels = 1; channels <= 4; ++channels) {
        Image8 img{37, 23, channels, {}};
        img.pixels.resize(std::size_t{37} * 23 * channels);
        for (std::uint32_t y = 0; y < 23; ++y) {
            for (std::uint32_t x = 0; x < 37; ++x) {
                for (std::uint32_t c = 0; c < channels; ++c) {
                    img.pixels[(y * 37 + x) * channels + c] = static_cast<std::uint8_t>(x * 5 + y * 3 + c * 40 + ((x * y) % 7));
                }
            }
        }
        const Bytes encoded = encode_png(img);
        const auto decoded = decode_png(encoded);
        REQUIRE(decoded.has_value());
        CHECK_EQ(decoded->width, 37U);
        CHECK_EQ(decoded->height, 23U);
        CHECK_EQ(decoded->channels, channels);
        CHECK(decoded->pixels == img.pixels);
    }
}

TEST_CASE(png, decodes_independently_produced_file_using_all_filters) {
    // 6x5 RGB, rows use filters 0..4 (None, Sub, Up, Average, Paeth); generated outside this code base.
    const Bytes file = {137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82,0,0,0,6,0,0,0,5,8,2,0,0,0,233,58,10,177,0,0,0,106,73,68,65,84,120,218,1,95,0,160,255,0,130,183,14,238,127,26,80,57,190,240,126,194,52,127,6,110,208,143,1,93,199,81,199,128,146,28,252,29,194,40,110,82,234,38,76,16,212,2,0,157,115,116,196,213,148,17,74,133,182,59,70,172,25,129,57,77,3,110,196,63,196,118,69,193,120,230,155,152,56,183,122,254,171,74,47,4,123,102,195,37,33,75,116,152,253,146,128,238,171,219,34,110,102,199,255,63,44,37,98,222,152,243,0,0,0,0,73,69,78,68,174,66,96,130};
    const Bytes expected = {130,183,14,238,127,26,80,57,190,240,126,194,52,127,6,110,208,143,93,199,81,36,71,227,64,67,0,2,107,110,84,85,148,160,101,104,93,100,196,152,11,184,212,84,74,135,33,169,154,1,173,33,158,181,156,246,161,94,246,241,90,29,131,11,183,206,9,214,187,192,4,231,23,92,100,60,125,236,176,181,128,236,55,188,151,18,221,46,106,174};
    const auto img = decode_png(file);
    REQUIRE(img.has_value());
    CHECK_EQ(img->width, 6U);
    CHECK_EQ(img->height, 5U);
    CHECK(img->pixels == expected);
}

TEST_CASE(png, encoder_output_has_valid_structure_and_compresses_smooth_images) {
    Image8 img{256, 128, 3, {}};
    img.pixels.resize(256 * 128 * 3);
    for (std::uint32_t y = 0; y < 128; ++y) {
        for (std::uint32_t x = 0; x < 256; ++x) {
            img.pixels[(y * 256 + x) * 3 + 0] = static_cast<std::uint8_t>(x);
            img.pixels[(y * 256 + x) * 3 + 1] = static_cast<std::uint8_t>(y * 2);
            img.pixels[(y * 256 + x) * 3 + 2] = 128;
        }
    }
    const Bytes encoded = encode_png(img);
    CHECK(encoded.size() < img.pixels.size() / 10);
    CHECK_EQ(encoded[1], static_cast<std::uint8_t>('P'));
    CHECK_EQ(encoded[12], static_cast<std::uint8_t>('I'));  // first chunk is IHDR
    CHECK_EQ(encoded[encoded.size() - 8], static_cast<std::uint8_t>('I'));  // IEND
}

TEST_CASE(png, rejects_malformed_input) {
    Image8 img{8, 8, 3, Bytes(8 * 8 * 3, 10)};
    const Bytes good = encode_png(img);
    CHECK(decode_png(good).has_value());
    CHECK(!decode_png({}));
    CHECK(!decode_png(bytes_of("not a png at all, really")));
    Bytes bad_crc = good;
    bad_crc[20] ^= 1;
    CHECK(!decode_png(bad_crc));
    CHECK(!decode_png(std::span(good).first(good.size() - 5)));
    Bytes wrong_idat = good;  // flip a byte inside IDAT; CRC catches it
    wrong_idat[good.size() - 20] ^= 0x10;
    CHECK(!decode_png(wrong_idat));
    CHECK_THROWS(encode_png(Image8{4, 4, 3, Bytes(5)}));
    CHECK_THROWS(encode_png(Image8{0, 4, 3, {}}));
}

TEST_CASE(png, file_round_trip_and_missing_file) {
    const auto path = (std::filesystem::temp_directory_path() / "lumen_png_test.png").string();
    Image8 img{5, 4, 4, Bytes(5 * 4 * 4, 99)};
    write_png_file(path, img);
    const auto back = read_png_file(path);
    REQUIRE(back.has_value());
    CHECK(back->pixels == img.pixels);
    std::remove(path.c_str());
    CHECK(!read_png_file(path));
    CHECK_THROWS(write_png_file("/nonexistent-dir-for-lumen/out.png", img));
}
