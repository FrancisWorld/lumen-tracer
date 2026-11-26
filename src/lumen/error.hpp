#pragma once

#include <stdexcept>
#include <string>

namespace lumen {

/// Raised for invalid user input (JSON syntax, scene content, OBJ files, I/O).
/// Messages are human-readable and carry the location of the problem.
class InputError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace lumen
