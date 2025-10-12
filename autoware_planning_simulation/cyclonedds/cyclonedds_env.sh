#!/bin/bash
# CycloneDDS environment setup for Autoware planning simulation

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Workspace is three levels up (cyclonedds -> autoware_planning_simulation -> autoware-rmw-zenoh-benchmark -> 2025.02-ws)
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

export ROS_DOMAIN_ID=188
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source Autoware workspace setup
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    . "$WORKSPACE_DIR/install/setup.bash"
else
    echo "WARNING: Autoware workspace setup not found at $WORKSPACE_DIR/install/setup.bash"
fi

# Execute the command passed as arguments, or start a shell if none
if [ $# -eq 0 ]; then
    echo "Environment configured for CycloneDDS:"
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
    exec bash
else
    exec "$@"
fi
