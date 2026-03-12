#!/usr/bin/env bash
set -euo pipefail

BUILD_TYPE="${1:-Release}"
BUILD_DIR="build/${BUILD_TYPE}"

cmake -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DBUILD_TESTS=ON

cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

echo ""
echo "Build complete: ${BUILD_DIR}/src/vision"