#!/bin/bash
set -e

# Configuration
PACKAGE_NAME="hector_testing_utils"
WORKSPACE_DIR=$(pwd)
BUILD_DIR="${WORKSPACE_DIR}/build/${PACKAGE_NAME}"
OUTPUT_DIR="${WORKSPACE_DIR}/coverage_report"

echo "----------------------------------------------------------------"
echo "Generating Coverage for ${PACKAGE_NAME}"
echo "----------------------------------------------------------------"

# 1. Clean previous build (ensure fresh counters)
echo "[1/5] Cleaning build directory..."
rm -rf "${BUILD_DIR}"
# Also clean install to be safe, though mainly build matters for coverage artifacts
rm -rf "${WORKSPACE_DIR}/install/${PACKAGE_NAME}"

# 2. Build with Coverage Flags
echo "[2/5] Building with ENABLE_COVERAGE_TESTING=ON..."
colcon build --packages-select ${PACKAGE_NAME} \
  --cmake-args -DENABLE_COVERAGE_TESTING=ON \
  --event-handlers console_direct+

# 3. Clean previous lcov data
echo "[3/5] Resetting lcov counters..."
lcov --directory "${BUILD_DIR}" --zerocounters -q

# 4. Run Tests
echo "[4/5] Running tests..."
colcon test --packages-select ${PACKAGE_NAME} || true # Allow failure to capture coverage of failures too

# 5. Capture and Generate Report
echo "[5/5] Generating HTML report..."

# Capture initial coverage data
lcov --directory "${BUILD_DIR}" --capture --output-file coverage.info --ignore-errors mismatch

# Filter out system headers, test files, and build artifacts
# We only want src/include coverage
lcov --remove coverage.info \
    '/usr/*' \
    '*/test/*' \
    '*/build/*' \
    '*/install/*' \
    --output-file coverage_filtered.info \
    --ignore-errors unused

# Generate HTML
mkdir -p "${OUTPUT_DIR}"
genhtml coverage_filtered.info --output-directory "${OUTPUT_DIR}" --legend

echo "----------------------------------------------------------------"
echo "Coverage Report Generated: file://${OUTPUT_DIR}/index.html"
echo "----------------------------------------------------------------"
