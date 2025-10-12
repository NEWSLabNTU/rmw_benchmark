#!/bin/bash
# CycloneDDS environment setup for Autoware planning simulation

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Get benchmark root directory
BENCHMARK_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
# Autoware workspace is via symlink
AUTOWARE_DIR="$SCRIPT_DIR/../autoware"

export ROS_DOMAIN_ID=188
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source Autoware workspace setup (via symlink)
if [ -L "$AUTOWARE_DIR" ] && [ -f "$AUTOWARE_DIR/install/setup.bash" ]; then
    . "$AUTOWARE_DIR/install/setup.bash"
else
    echo "ERROR: Autoware symlink not set up correctly at $AUTOWARE_DIR"
    echo "Please update the symlink to point to your Autoware workspace:"
    echo "  ln -sfn /path/to/your/autoware/install $SCRIPT_DIR/../autoware"
    exit 1
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
