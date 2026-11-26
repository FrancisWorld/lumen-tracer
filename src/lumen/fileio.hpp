#pragma once

#include <cstddef>
#include <string>

namespace lumen {

inline constexpr std::size_t kMaxSceneBytes = std::size_t{64} << 20;
inline constexpr std::size_t kMaxAssetBytes = std::size_t{256} << 20;

/// Reads a whole file, refusing anything larger than `max_bytes` *before* it is loaded into memory.
/// The size is checked up front and again while reading, so special files and files that grow
/// are bounded too. Throws InputError.
std::string read_file_bounded(const std::string& path, std::size_t max_bytes);

}  // namespace lumen
