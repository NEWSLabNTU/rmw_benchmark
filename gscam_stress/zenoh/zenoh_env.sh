#!/bin/bash
# Zenoh environment setup for gscam stress test

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# gscam_stress root is parent directory
GSCAM_STRESS_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
# Get benchmark root directory
BENCHMARK_ROOT="$( cd "$GSCAM_STRESS_DIR/.." && pwd )"

export ROS_DOMAIN_ID=189
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG="file://$GSCAM_STRESS_DIR/rmw_config/zenoh_shm.json5"
export RUST_LOG=zenoh=info

# Source gscam_stress workspace setup if it exists
if [ -f "$GSCAM_STRESS_DIR/install/setup.bash" ]; then
    . "$GSCAM_STRESS_DIR/install/setup.bash"
else
    echo "WARNING: gscam_stress not built yet. Run 'make build' in $GSCAM_STRESS_DIR"
fi

# Source rmw_zenoh workspace setup (from submodule)
RMW_ZENOH_INSTALL="$BENCHMARK_ROOT/common/rmw_zenoh/install"
if [ -f "$RMW_ZENOH_INSTALL/setup.bash" ]; then
    . "$RMW_ZENOH_INSTALL/setup.bash"
else
    echo "WARNING: rmw_zenoh not built yet. Run 'make build-rmw-zenoh' in $BENCHMARK_ROOT"
fi

# Execute the command passed as arguments, or start a shell if none
if [ $# -eq 0 ]; then
    echo "Environment configured for Zenoh stress test:"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    echo "  ZENOH_CONFIG=$ZENOH_CONFIG"
    echo "  RUST_LOG=$RUST_LOG"
    exec bash
else
    exec "$@"
fi
