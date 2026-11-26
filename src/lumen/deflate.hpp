// zlib container (RFC 1950) with a hand-written DEFLATE (RFC 1951) codec.
// The compressor emits LZ77 + fixed Huffman blocks; the decompressor accepts
// every block type (stored, fixed, dynamic) so it can read third-party PNGs.
#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace lumen {

using Bytes = std::vector<std::uint8_t>;

std::uint32_t crc32(std::span<const std::uint8_t> data, std::uint32_t seed = 0);
std::uint32_t adler32(std::span<const std::uint8_t> data);

Bytes zlib_compress(std::span<const std::uint8_t> data);
/// `max_output` bounds the decoded size so hostile streams cannot exhaust memory.
std::expected<Bytes, std::string> zlib_decompress(std::span<const std::uint8_t> data, std::size_t max_output);

}  // namespace lumen
