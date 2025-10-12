#!/bin/bash
#
# RMW Boundary Test Suite Runner
# Systematically tests both CycloneDDS and Zenoh across multiple configurations
# to find frame drop boundaries
#
# Usage: ./run_boundary_test.sh [duration_per_test]
#

set -e -o pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Test duration per config (default: 60 seconds)
DURATION="${1:-60}"

# Results directory
RESULTS_DIR="$PROJECT_ROOT/results"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
TEST_RUN_DIR="$RESULTS_DIR/run_$TIMESTAMP"

# Functions
print_header() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

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

# Test configurations in order of increasing load
CONFIGS=(
    "640x480_30fps.conf"
    "low.conf"                # 640x480@15fps
    "medium.conf"             # 1280x720@30fps
    "1280x720_45fps.conf"
    "1280x720_60fps.conf"
    "high.conf"               # 1920x1080@30fps
    "1920x1080_45fps.conf"
    "1080p60.conf"            # 1920x1080@60fps
    "2560x1440_30fps.conf"
    "2560x1440_45fps.conf"
    "2560x1440_60fps.conf"
    "extreme.conf"            # 3840x2160@30fps
    "3840x2160_45fps.conf"
    "3840x2160_60fps.conf"
)

# Create results directory structure
mkdir -p "$TEST_RUN_DIR"/{cyclonedds,zenoh}

# Create summary CSV
SUMMARY_CSV="$TEST_RUN_DIR/summary.csv"
echo "rmw_type,config,resolution,fps,data_rate_mbps,avg_frame_loss_pct,max_frame_loss_pct,avg_latency_ms,max_latency_ms,avg_message_rate,test_duration_sec,timestamp" > "$SUMMARY_CSV"

print_header "RMW Boundary Test Suite"
print_info "Test duration per config: ${DURATION}s"
print_info "Number of configs: ${#CONFIGS[@]}"
print_info "Total tests: $((${#CONFIGS[@]} * 2)) (CycloneDDS + Zenoh)"
print_info "Estimated time: ~$(( (${#CONFIGS[@]} * 2 * (DURATION + 10)) / 60 )) minutes"
print_info "Results directory: $TEST_RUN_DIR"
echo ""

# Function to run test and collect results
run_test() {
    local rmw_type=$1
    local config=$2
    local test_num=$3
    local total_tests=$4

    print_header "Test $test_num/$total_tests: $rmw_type - $config"

    # Run benchmark
    if ! "$SCRIPT_DIR/benchmark_rmw.sh" "$rmw_type" "$config" "$DURATION"; then
        print_error "Test failed: $rmw_type - $config"
        return 1
    fi

    # Copy CSV to results directory
    CSV_DIR="$PROJECT_ROOT/$rmw_type"
    LATEST_CSV=$(ls -t "$CSV_DIR"/*.csv 2>/dev/null | head -1)

    if [ -n "$LATEST_CSV" ] && [ -f "$LATEST_CSV" ]; then
        # Copy CSV with descriptive name
        CONFIG_BASE=$(basename "$config" .conf)
        DEST_CSV="$TEST_RUN_DIR/$rmw_type/${CONFIG_BASE}_$(basename "$LATEST_CSV")"
        cp "$LATEST_CSV" "$DEST_CSV"
        print_success "Results saved: $DEST_CSV"

        # Extract summary statistics
        AVG_LOSS=$(awk -F',' 'NR>1 {sum+=$7; count++} END {if(count>0) printf "%.2f", sum/count; else print "N/A"}' "$LATEST_CSV")
        MAX_LOSS=$(awk -F',' 'NR>1 {if($7>max || max=="") max=$7} END {if(max!="") printf "%.2f", max; else print "N/A"}' "$LATEST_CSV")
        AVG_LATENCY=$(awk -F',' 'NR>1 {sum+=$8; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}' "$LATEST_CSV")
        MAX_LATENCY=$(awk -F',' 'NR>1 {if($9>max || max=="") max=$9} END {if(max!="") printf "%.3f", max; else print "N/A"}' "$LATEST_CSV")
        AVG_RATE=$(awk -F',' 'NR>1 {sum+=$6; count++} END {if(count>0) printf "%.2f", sum/count; else print "N/A"}' "$LATEST_CSV")

        # Get config parameters from first data line
        WIDTH=$(awk -F',' 'NR==2 {print $2}' "$LATEST_CSV")
        HEIGHT=$(awk -F',' 'NR==2 {print $3}' "$LATEST_CSV")
        FPS=$(awk -F',' 'NR==2 {print $4}' "$LATEST_CSV")
        RESOLUTION="${WIDTH}x${HEIGHT}"

        # Calculate data rate (MB/s)
        DATA_RATE_MBPS=$(echo "scale=1; $WIDTH * $HEIGHT * 3 * $FPS / 1048576" | bc)

        # Get test timestamp
        TEST_TIMESTAMP=$(basename "$LATEST_CSV" .csv)

        # Append to summary CSV
        echo "$rmw_type,$CONFIG_BASE,$RESOLUTION,$FPS,$DATA_RATE_MBPS,$AVG_LOSS,$MAX_LOSS,$AVG_LATENCY,$MAX_LATENCY,$AVG_RATE,$DURATION,$TEST_TIMESTAMP" >> "$SUMMARY_CSV"
    fi

    # Brief pause between tests
    sleep 3
}

# Run all tests
TOTAL_TESTS=$((${#CONFIGS[@]} * 2))
TEST_NUM=0

# Test CycloneDDS
print_header "Phase 1: Testing CycloneDDS"
for config in "${CONFIGS[@]}"; do
    TEST_NUM=$((TEST_NUM + 1))
    if ! run_test "cyclonedds" "$config" "$TEST_NUM" "$TOTAL_TESTS"; then
        print_warning "Skipping remaining CycloneDDS tests due to failure"
        break
    fi
done

# Brief pause between RMW implementations
sleep 5

# Test Zenoh
print_header "Phase 2: Testing Zenoh"
for config in "${CONFIGS[@]}"; do
    TEST_NUM=$((TEST_NUM + 1))
    if ! run_test "zenoh" "$config" "$TEST_NUM" "$TOTAL_TESTS"; then
        print_warning "Skipping remaining Zenoh tests due to failure"
        break
    fi
done

# Generate final report
print_header "Test Suite Complete!"
print_success "All tests completed"
print_info "Results saved in: $TEST_RUN_DIR"
print_info "Summary CSV: $SUMMARY_CSV"
echo ""

# Display boundary analysis
print_header "Boundary Analysis"

# Analyze CycloneDDS boundaries
print_info "=== CycloneDDS Frame Drop Boundaries ==="
awk -F',' '
    NR>1 && $1=="cyclonedds" {
        loss = $6
        config = $2
        resolution = $3
        fps = $4
        datarate = $5

        if (loss < 0.1 && !found_01) {
            printf "  No loss (<0.1%%):     %s (%s@%dfps, ~%.0f MB/s)\n", config, resolution, fps, datarate
        }
        if (loss >= 0.1 && loss < 1.0 && !found_1) {
            printf "  Minor drops (0.1-1%%): %s (%s@%dfps, ~%.0f MB/s, %.2f%% loss)\n", config, resolution, fps, datarate, loss
            found_1 = 1
        }
        if (loss >= 1.0 && loss < 5.0 && !found_5) {
            printf "  Moderate drops (1-5%%): %s (%s@%dfps, ~%.0f MB/s, %.2f%% loss)\n", config, resolution, fps, datarate, loss
            found_5 = 1
        }
        if (loss >= 5.0 && !found_severe) {
            printf "  Severe drops (>5%%):   %s (%s@%dfps, ~%.0f MB/s, %.2f%% loss)\n", config, resolution, fps, datarate, loss
            found_severe = 1
        }
    }
' "$SUMMARY_CSV"

echo ""

# Analyze Zenoh boundaries
print_info "=== Zenoh Frame Drop Boundaries ==="
awk -F',' '
    NR>1 && $1=="zenoh" {
        loss = $6
        config = $2
        resolution = $3
        fps = $4
        datarate = $5

        if (loss < 0.1 && !found_01) {
            printf "  No loss (<0.1%%):     %s (%s@%dfps, ~%.0f MB/s)\n", config, resolution, fps, datarate
        }
        if (loss >= 0.1 && loss < 1.0 && !found_1) {
            printf "  Minor drops (0.1-1%%): %s (%s@%dfps, ~%.0f MB/s, %.2f%% loss)\n", config, resolution, fps, datarate, loss
            found_1 = 1
        }
        if (loss >= 1.0 && loss < 5.0 && !found_5) {
            printf "  Moderate drops (1-5%%): %s (%s@%dfps, ~%.0f MB/s, %.2f%% loss)\n", config, resolution, fps, datarate, loss
            found_5 = 1
        }
        if (loss >= 5.0 && !found_severe) {
            printf "  Severe drops (>5%%):   %s (%s@%dfps, ~%.0f MB/s, %.2f%% loss)\n", config, resolution, fps, datarate, loss
            found_severe = 1
        }
    }
' "$SUMMARY_CSV"

echo ""
print_info "=========================================="
print_info "For detailed analysis, see: $SUMMARY_CSV"
print_info "=========================================="
