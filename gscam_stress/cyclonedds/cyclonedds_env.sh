#!/bin/bash
# CycloneDDS environment setup for gscam stress test

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# gscam_stress root is parent directory
GSCAM_STRESS_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
# Workspace is three levels up (gscam_stress -> autoware-rmw-zenoh-benchmark -> 2025.02-ws)
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

export ROS_DOMAIN_ID=188
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$GSCAM_STRESS_DIR/src/stress_test/config/cyclonedds_shm.xml"

# Source Autoware workspace setup
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    . "$WORKSPACE_DIR/install/setup.bash"
else
    echo "WARNING: Autoware workspace setup not found at $WORKSPACE_DIR/install/setup.bash"
fi

# Source gscam_stress workspace setup if it exists
if [ -f "$GSCAM_STRESS_DIR/install/setup.bash" ]; then
    . "$GSCAM_STRESS_DIR/install/setup.bash"
else
    echo "WARNING: gscam_stress not built yet. Run 'make build' in $GSCAM_STRESS_DIR"
fi

# Execute the command passed as arguments, or start a shell if none
if [ $# -eq 0 ]; then
    echo "Environment configured for CycloneDDS stress test:"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    echo "  CYCLONEDDS_URI=$CYCLONEDDS_URI"
    exec bash
else
    exec "$@"
fi
