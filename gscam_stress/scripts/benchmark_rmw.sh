#!/bin/bash
#
# RMW Benchmark Automation Script
# Automates service startup, data collection, and shutdown
#
# Usage: ./benchmark_rmw.sh <rmw_type> <config> <duration_sec>
# Example: ./benchmark_rmw.sh cyclonedds high.conf 60
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to display usage
usage() {
    cat <<EOF
Usage: $0 <rmw_type> <config> <duration_sec>

Arguments:
  rmw_type      RMW implementation: cyclonedds or zenoh
  config        Configuration file name (e.g., high.conf)
  duration_sec  Test duration in seconds (default: 60)

Examples:
  $0 cyclonedds medium.conf 60
  $0 zenoh 1920x1080_45fps.conf 120

EOF
    exit 1
}

# Check arguments
if [ $# -lt 2 ]; then
    usage
fi

RMW_TYPE="$1"
CONFIG="$2"
DURATION="${3:-60}"

# Validate RMW type
if [ "$RMW_TYPE" != "cyclonedds" ] && [ "$RMW_TYPE" != "zenoh" ]; then
    print_error "Invalid RMW type: $RMW_TYPE"
    print_error "Must be 'cyclonedds' or 'zenoh'"
    exit 1
fi

# Check if config file exists
CONFIG_PATH="$PROJECT_ROOT/src/stress_test/config/$CONFIG"
if [ ! -f "$CONFIG_PATH" ]; then
    print_error "Config file not found: $CONFIG_PATH"
    exit 1
fi

print_info "=========================================="
print_info "RMW Benchmark Test"
print_info "=========================================="
print_info "RMW Type:    $RMW_TYPE"
print_info "Config:      $CONFIG"
print_info "Duration:    ${DURATION}s"
print_info "=========================================="

# Change to project root directory
cd "$PROJECT_ROOT"

# Stop any existing services
print_info "Stopping existing services..."
make stop-all 2>/dev/null || true

# For Zenoh, start router first
if [ "$RMW_TYPE" = "zenoh" ]; then
    print_info "Starting Zenoh router..."
    make start-router
    sleep 2

    # Verify router is running
    if ! systemctl --user is-active --quiet ros2-stress-test-zenoh-router.service 2>/dev/null; then
        print_error "Failed to start Zenoh router"
        exit 1
    fi
    print_success "Zenoh router started"
fi

# Start stress test
print_info "Starting stress test with $RMW_TYPE..."
if [ "$RMW_TYPE" = "cyclonedds" ]; then
    STRESS_CONFIG="$CONFIG" make start-cyclonedds
else
    STRESS_CONFIG="$CONFIG" make start-zenoh
fi

sleep 2

# Verify service is running
SERVICE_NAME="stress-test-${RMW_TYPE}"
if ! systemctl --user is-active --quiet "ros2-${SERVICE_NAME}.service" 2>/dev/null; then
    print_error "Failed to start stress test service"
    make stop-all
    exit 1
fi

print_success "Stress test started"

# Get the CSV file path
CSV_DIR="$PROJECT_ROOT/$RMW_TYPE"
LATEST_CSV=$(ls -t "$CSV_DIR"/*.csv 2>/dev/null | head -1)

if [ -n "$LATEST_CSV" ]; then
    print_info "CSV output: $LATEST_CSV"
fi

# Monitor test progress
print_info "Collecting data for ${DURATION} seconds..."
for ((i=1; i<=DURATION; i++)); do
    if [ $((i % 10)) -eq 0 ]; then
        print_info "Progress: ${i}/${DURATION}s"
    fi
    sleep 1

    # Check if service is still running
    if ! systemctl --user is-active --quiet "ros2-${SERVICE_NAME}.service" 2>/dev/null; then
        print_error "Service stopped unexpectedly!"
        break
    fi
done

# Stop services
print_info "Stopping services..."
make stop-all

sleep 1

print_success "Test completed!"

# Analyze results if CSV exists
if [ -n "$LATEST_CSV" ] && [ -f "$LATEST_CSV" ]; then
    print_info "=========================================="
    print_info "Results Summary"
    print_info "=========================================="

    # Parse CSV to get statistics
    if [ -f "$LATEST_CSV" ]; then
        # Skip header, calculate average frame loss
        AVG_LOSS=$(awk -F',' 'NR>1 {sum+=$7; count++} END {if(count>0) printf "%.2f", sum/count}' "$LATEST_CSV")
        MAX_LOSS=$(awk -F',' 'NR>1 {if($7>max || max=="") max=$7} END {printf "%.2f", max}' "$LATEST_CSV")
        AVG_LATENCY=$(awk -F',' 'NR>1 {sum+=$8; count++} END {if(count>0) printf "%.3f", sum/count}' "$LATEST_CSV")
        MAX_LATENCY=$(awk -F',' 'NR>1 {if($9>max || max=="") max=$9} END {printf "%.3f", max}' "$LATEST_CSV")
        AVG_RATE=$(awk -F',' 'NR>1 {sum+=$6; count++} END {if(count>0) printf "%.2f", sum/count}' "$LATEST_CSV")

        # Get config parameters from first data line
        CONFIG_INFO=$(awk -F',' 'NR==2 {printf "%dx%d@%dfps", $2, $3, $4}' "$LATEST_CSV")

        print_info "Configuration: $CONFIG_INFO"
        print_info "Average message rate: ${AVG_RATE} msg/s"
        print_info "Frame loss: avg ${AVG_LOSS}%, max ${MAX_LOSS}%"
        print_info "Latency: avg ${AVG_LATENCY}ms, max ${MAX_LATENCY}ms"

        # Color-coded frame loss assessment
        if (( $(echo "$AVG_LOSS < 0.1" | bc -l) )); then
            print_success "Frame loss: EXCELLENT (< 0.1%)"
        elif (( $(echo "$AVG_LOSS < 1.0" | bc -l) )); then
            print_warning "Frame loss: GOOD (< 1%)"
        elif (( $(echo "$AVG_LOSS < 5.0" | bc -l) )); then
            print_warning "Frame loss: MODERATE (1-5%)"
        else
            print_error "Frame loss: SEVERE (> 5%)"
        fi
    fi

    print_info "Full results: $LATEST_CSV"
fi

print_info "=========================================="
