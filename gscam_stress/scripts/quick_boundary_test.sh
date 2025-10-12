#!/bin/bash
# Quick boundary test with key configurations

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RESULTS_DIR="$PROJECT_ROOT/results"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
TEST_RUN_DIR="$RESULTS_DIR/quick_run_$TIMESTAMP"

mkdir -p "$TEST_RUN_DIR"/{cyclonedds,zenoh}

SUMMARY_CSV="$TEST_RUN_DIR/summary.csv"
echo "rmw_type,config,resolution,fps,data_rate_mbps,avg_frame_loss_pct,max_frame_loss_pct,avg_latency_ms,max_latency_ms,avg_message_rate,test_duration_sec,timestamp" > "$SUMMARY_CSV"

# Key configurations to test
CONFIGS=(
    "low.conf"
    "medium.conf"
    "high.conf"
    "1080p60.conf"
    "2560x1440_30fps.conf"
    "2560x1440_60fps.conf"
)

DURATION=30

echo "Quick Boundary Test"
echo "===================="
echo "Configs: ${#CONFIGS[@]}"
echo "Duration per test: ${DURATION}s"
echo "Results: $TEST_RUN_DIR"
echo ""

test_num=0
total_tests=$((${#CONFIGS[@]} * 2))

for config in "${CONFIGS[@]}"; do
    for rmw in cyclonedds zenoh; do
        ((test_num++))
        echo "[$test_num/$total_tests] Testing $rmw - $config"

        if "$SCRIPT_DIR/benchmark_rmw.sh" "$rmw" "$config" "$DURATION" > /tmp/bench_output.log 2>&1; then
            # Copy CSV
            CSV_DIR="$PROJECT_ROOT/$rmw"
            LATEST_CSV=$(ls -t "$CSV_DIR"/*.csv 2>/dev/null | head -1)
            
            if [ -n "$LATEST_CSV" ] && [ -f "$LATEST_CSV" ]; then
                CONFIG_BASE=$(basename "$config" .conf)
                cp "$LATEST_CSV" "$TEST_RUN_DIR/$rmw/${CONFIG_BASE}_$(basename "$LATEST_CSV")"
                
                # Extract stats and append to summary
                AVG_LOSS=$(awk -F',' 'NR>1 {sum+=$7; count++} END {if(count>0) printf "%.2f", sum/count; else print "0.00"}' "$LATEST_CSV")
                MAX_LOSS=$(awk -F',' 'NR>1 {if($7>max || max=="") max=$7} END {if(max!="") printf "%.2f", max; else print "0.00"}' "$LATEST_CSV")
                AVG_LATENCY=$(awk -F',' 'NR>1 {sum+=$8; count++} END {if(count>0) printf "%.3f", sum/count; else print "0.000"}' "$LATEST_CSV")
                MAX_LATENCY=$(awk -F',' 'NR>1 {if($9>max || max=="") max=$9} END {if(max!="") printf "%.3f", max; else print "0.000"}' "$LATEST_CSV")
                AVG_RATE=$(awk -F',' 'NR>1 {sum+=$6; count++} END {if(count>0) printf "%.2f", sum/count; else print "0.00"}' "$LATEST_CSV")
                WIDTH=$(awk -F',' 'NR==2 {print $2}' "$LATEST_CSV")
                HEIGHT=$(awk -F',' 'NR==2 {print $3}' "$LATEST_CSV")
                FPS=$(awk -F',' 'NR==2 {print $4}' "$LATEST_CSV")
                RESOLUTION="${WIDTH}x${HEIGHT}"
                DATA_RATE_MBPS=$(echo "scale=1; $WIDTH * $HEIGHT * 3 * $FPS / 1048576" | bc)
                TEST_TIMESTAMP=$(basename "$LATEST_CSV" .csv)
                
                echo "$rmw,$CONFIG_BASE,$RESOLUTION,$FPS,$DATA_RATE_MBPS,$AVG_LOSS,$MAX_LOSS,$AVG_LATENCY,$MAX_LATENCY,$AVG_RATE,$DURATION,$TEST_TIMESTAMP" >> "$SUMMARY_CSV"
                echo "  Loss: ${AVG_LOSS}%, Latency: ${AVG_LATENCY}ms"
            fi
        else
            echo "  FAILED"
        fi
        
        sleep 2
    done
done

echo ""
echo "Quick test complete!"
echo "Results: $TEST_RUN_DIR"
echo "Summary: $SUMMARY_CSV"
