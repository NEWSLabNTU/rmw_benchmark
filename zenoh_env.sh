#!/bin/bash
# Zenoh environment setup wrapper script

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Workspace is one level up from script directory
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

export ROS_DOMAIN_ID=189
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Source setup files from workspace root
. "$WORKSPACE_DIR/install/setup.sh"
. "$WORKSPACE_DIR/rmw_zenoh_ws/install/setup.sh"

# Execute the command passed as arguments
exec "$@"
