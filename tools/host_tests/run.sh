#!/bin/sh
# Build and run the host-side GNSS parser tests. No board, no receiver needed.
#
#   ./tools/host_tests/run.sh
set -e
cd "$(dirname "$0")/../.."

OUT=.pio/host-tests
mkdir -p "$OUT"

# Two binaries: the parser (real src/gnss.cpp + Arduino shim) and the mark layer
# (real src/waypoints.cpp, which needs no shim).
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter \
    -I tools/host_tests -I src \
    -o "$OUT/gnss_host_test" tools/host_tests/host_test.cpp

g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter \
    -I src \
    -o "$OUT/waypoint_host_test" tools/host_tests/waypoint_test.cpp

"$OUT/gnss_host_test"
"$OUT/waypoint_host_test"
