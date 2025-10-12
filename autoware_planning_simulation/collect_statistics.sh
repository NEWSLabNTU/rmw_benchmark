#!/bin/bash
# Script to collect ROS 2 node/topic/service statistics
# Usage: ./collect_statistics.sh <output_dir> <domain_id> <rmw_implementation>

set -e

# Check arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <output_dir> <domain_id> <rmw_implementation>"
    echo "Example: $0 cyclonedds_baseline 188 rmw_cyclonedds_cpp"
    exit 1
fi

OUTPUT_DIR="$1"
DOMAIN_ID="$2"
RMW_IMPLEMENTATION="$3"

# Get workspace directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Setup environment
export ROS_DOMAIN_ID="$DOMAIN_ID"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"
source "$WORKSPACE_DIR/install/setup.bash"

# Source Zenoh workspace if using Zenoh
if [ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]; then
    source "$WORKSPACE_DIR/rmw_zenoh_ws/install/setup.bash"
fi

echo "=========================================="
echo "Collecting ROS 2 Statistics"
echo "=========================================="
echo "Output Directory: $OUTPUT_DIR"
echo "ROS Domain ID: $DOMAIN_ID"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "=========================================="
echo ""

# Collect nodes
echo "Collecting nodes..."
ros2 node list > "$OUTPUT_DIR/nodes.txt" 2>&1
NODE_COUNT=$(grep -v '^WARNING' "$OUTPUT_DIR/nodes.txt" | wc -l)
echo "  ✓ Nodes collected: $NODE_COUNT"

# Collect topics
echo "Collecting topics with types..."
ros2 topic list -t > "$OUTPUT_DIR/topics.txt" 2>&1
TOPIC_COUNT=$(wc -l < "$OUTPUT_DIR/topics.txt")
echo "  ✓ Topics collected: $TOPIC_COUNT"

# Collect services
echo "Collecting services with types..."
ros2 service list -t > "$OUTPUT_DIR/services.txt" 2>&1
SERVICE_COUNT=$(wc -l < "$OUTPUT_DIR/services.txt")
echo "  ✓ Services collected: $SERVICE_COUNT"

# Check for duplicate nodes
echo ""
echo "Checking for duplicate nodes..."
DUPLICATES=$(grep -v '^WARNING' "$OUTPUT_DIR/nodes.txt" | sort | uniq -d)
if [ -z "$DUPLICATES" ]; then
    echo "  ✓ No duplicate nodes found"
else
    DUPLICATE_COUNT=$(echo "$DUPLICATES" | wc -l)
    echo "  ⚠ Found $DUPLICATE_COUNT duplicate nodes:"
    echo "$DUPLICATES" | sed 's/^/    /'
fi

# Summary
echo ""
echo "=========================================="
echo "Collection Complete!"
echo "=========================================="
echo "Statistics:"
echo "  Nodes:    $NODE_COUNT"
echo "  Topics:   $TOPIC_COUNT"
echo "  Services: $SERVICE_COUNT"
echo ""
echo "Files saved to: $OUTPUT_DIR/"
echo "  - nodes.txt"
echo "  - topics.txt"
echo "  - services.txt"
echo "=========================================="
