#!/bin/sh
# Source coverage of src/lumen with LLVM tools (Xcode or any LLVM toolchain).
#   scripts/coverage.sh        -> per-file table on stdout
set -eu
cd "$(dirname "$0")/.."
cmake --preset coverage >/dev/null
cmake --build --preset coverage -j 2 >/dev/null
profraw=build/coverage/profraw
rm -rf "$profraw" && mkdir -p "$profraw"
LLVM_PROFILE_FILE="$PWD/$profraw/%p.profraw" ctest --preset coverage >/dev/null
run() { if command -v xcrun >/dev/null 2>&1; then xcrun "$@"; else "$@"; fi; }
run llvm-profdata merge -sparse "$profraw"/*.profraw -o build/coverage/all.profdata
run llvm-cov report build/coverage/tests/lumen_tests -object build/coverage/lumen \
  -instr-profile=build/coverage/all.profdata -ignore-filename-regex='tests/|tools/|src/main.cpp'
