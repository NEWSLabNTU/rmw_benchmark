#!/bin/bash
# CycloneDDS environment setup wrapper script

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Workspace is one level up from script directory
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

export ROS_DOMAIN_ID=188
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source setup files from workspace root
. "$WORKSPACE_DIR/install/setup.sh"

# Execute the command passed as arguments
exec "$@"
