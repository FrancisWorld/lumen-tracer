#include "lumen/fileio.hpp"

#include <filesystem>
#include <fstream>

#include "lumen/error.hpp"

namespace lumen {

std::string read_file_bounded(const std::string& path, std::size_t max_bytes) {
    std::error_code ec;
    const auto size = std::filesystem::file_size(path, ec);
    if (!ec && size > max_bytes) {
        throw InputError(path + ": file too large (" + std::to_string(size) + " bytes, limit " + std::to_string(max_bytes) + ")");
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw InputError("cannot open " + path);
    }
    std::string data;
    char buffer[1 << 16];
    while (in) {
        in.read(buffer, sizeof buffer);
        const auto got = static_cast<std::size_t>(in.gcount());
        if (data.size() + got > max_bytes) {
            throw InputError(path + ": file too large (limit " + std::to_string(max_bytes) + " bytes)");
        }
        data.append(buffer, got);
    }
    return data;
}

}  // namespace lumen
