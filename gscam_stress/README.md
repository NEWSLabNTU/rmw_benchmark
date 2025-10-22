# gscam Stress Test for RMW Implementations

A configurable stress test for comparing ROS 2 RMW implementations (CycloneDDS vs Zenoh) using high-resolution video streaming.

## Benchmark Results Summary

**Latest benchmark run**: `results/run_2025-10-13_00-01-58/`

### Performance Comparison: CycloneDDS vs Zenoh

Comprehensive testing across 14 configurations (13 MB/s to 1424 MB/s) with Reliable/Volatile QoS:

| Configuration   | Resolution | FPS | Data Rate | CycloneDDS Loss % | Zenoh Loss % | CycloneDDS Latency (ms) | Zenoh Latency (ms) | Winner                  |
|-----------------|------------|-----|-----------|-------------------|--------------|-------------------------|--------------------|-------------------------|
| low             | 640x480    | 15  | 13 MB/s   | 0.00%             | 0.00%        | 0.71                    | 1.01               | Tie (CycloneDDS faster) |
| 640x480_30fps   | 640x480    | 30  | 26 MB/s   | 0.00%             | 0.00%        | 0.64                    | 0.91               | Tie (CycloneDDS faster) |
| medium          | 1280x720   | 30  | 79 MB/s   | 0.86%             | 0.00%        | 1.60                    | 2.01               | **Zenoh**               |
| 1280x720_45fps  | 1280x720   | 45  | 119 MB/s  | 1.29%             | 0.00%        | 1.50                    | 1.92               | **Zenoh**               |
| 1280x720_60fps  | 1280x720   | 60  | 158 MB/s  | 0.62%             | 0.00%        | 1.49                    | 1.85               | **Zenoh**               |
| high            | 1920x1080  | 30  | 178 MB/s  | 0.70%             | 0.00%        | 3.44                    | 4.23               | **Zenoh**               |
| 1920x1080_45fps | 1920x1080  | 45  | 267 MB/s  | 0.29%             | 0.00%        | 3.38                    | 4.04               | **Zenoh**               |
| 1080p60         | 1920x1080  | 60  | 356 MB/s  | 0.38%             | 0.00%        | 3.28                    | 3.91               | **Zenoh**               |
| 2560x1440_30fps | 2560x1440  | 30  | 316 MB/s  | 0.38%             | 0.00%        | 6.06                    | 7.19               | **Zenoh**               |
| 2560x1440_45fps | 2560x1440  | 45  | 475 MB/s  | 0.57%             | 0.00%        | 5.94                    | 7.04               | **Zenoh**               |
| 2560x1440_60fps | 2560x1440  | 60  | 633 MB/s  | 0.19%             | 0.03%        | 5.92                    | 7.00               | Tie (both excellent)    |
| extreme         | 3840x2160  | 30  | 712 MB/s  | 0.27%             | 0.05%        | 16.89                   | 22.13              | **CycloneDDS**          |
| 3840x2160_45fps | 3840x2160  | 45  | 1068 MB/s | 0.65%             | 0.04%        | 17.29                   | 22.33              | **Zenoh**               |
| 3840x2160_60fps | 3840x2160  | 60  | 1424 MB/s | 0.83%             | 5.13%        | 18.44                   | 27.10              | **CycloneDDS**          |

### Key Findings

**Maximum Capability Analysis:**

1. **CycloneDDS**
   - Maintains 0.00-0.86% frame loss across all data rates (13-1424 MB/s)
   - Best performance at extreme loads: 4K@60fps (1424 MB/s) with only 0.83% loss
   - Consistent low latency even at maximum throughput (18.44ms @ 1424 MB/s)
   - **Max tested capability**: 1424 MB/s (4K@60fps) with <1% frame loss

2. **Zenoh**
   - Perfect 0.00% frame loss from 13 MB/s to 633 MB/s (2K@60fps)
   - Maintains excellent reliability up to 1068 MB/s (4K@45fps) with only 0.04% loss
   - Performance degradation at 4K@60fps: 5.13% frame loss at 1424 MB/s
   - **Max tested capability**: 1068 MB/s (4K@45fps) with <0.1% frame loss

**Reliability on Drop Rate:**

- **Low to Medium Load (13-158 MB/s)**:
  - CycloneDDS: 0.00-1.29% loss, lower latency
  - Zenoh: Perfect 0.00% loss, slightly higher latency
  - **Zenoh is more reliable** in this range (HD and below)

- **Medium to High Load (178-633 MB/s)**:
  - CycloneDDS: 0.19-0.70% loss
  - Zenoh: 0.00-0.03% loss
  - **Zenoh significantly more reliable** (Full HD to 2K)

- **Extreme Load (712-1424 MB/s)**:
  - CycloneDDS: 0.27-0.83% loss, consistent performance
  - Zenoh: 0.04-5.13% loss, degrades at maximum
  - **CycloneDDS more reliable** at extreme 4K loads

**Recommendations:**

- **For HD/Full HD streaming (<356 MB/s)**: Use Zenoh for zero frame loss
- **For 2K streaming (316-633 MB/s)**: Use Zenoh for superior reliability
- **For 4K@60fps (>1068 MB/s)**: Use CycloneDDS for consistent performance
- **For latency-critical applications**: CycloneDDS has lower latency at most resolutions

See `results/run_2025-10-13_00-01-58/summary.csv` for complete raw data.

## Automated Benchmark Suite

Run comprehensive automated tests to compare RMW implementations:

```bash
# Quick test: 6 key configs, ~8 minutes
make benchmark-quick

# Full suite: 14 configs, ~35 minutes
make benchmark

# Analyze latest results
make benchmark-analyze
```

All benchmark scripts are in `scripts/` directory. See [scripts/README.md](scripts/README.md) for details.

## Quick Start

```bash
# 1. Build the stress test
make build

# 2. View current configuration (default: medium.conf)
make show-config

# 3. List available configurations
make list-configs

# 4. Run test with CycloneDDS (using default medium.conf)
make start-cyclonedds

# 5. Monitor performance
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw

# 6. Stop test
make stop-cyclonedds

# 7. Try different config (e.g., high stress)
make STRESS_CONFIG=high.conf start-cyclonedds
# Monitor...
make stop-cyclonedds

# 8. Compare with Zenoh
make start-router
make STRESS_CONFIG=high.conf start-zenoh
# Monitor and compare...
make stop-all
```

## Configuration Profiles

Each stress test configuration is stored in a separate file in `src/stress_test/config/`. The default configuration is `medium.conf`.

### Available Configurations

#### Standard Configurations (for benchmarking)

| Config File              | Resolution | FPS | Data Rate  | QoS               | Use Case                      |
|--------------------------|------------|-----|------------|-------------------|-------------------------------|
| **low.conf**             | 640x480    | 15  | ~13 MB/s   | Reliable/Volatile | Initial testing, debugging    |
| **640x480_30fps.conf**   | 640x480    | 30  | ~26 MB/s   | Reliable/Volatile | Low load baseline             |
| **medium.conf**          | 1280x720   | 30  | ~79 MB/s   | Reliable/Volatile | Standard HD (default)         |
| **1280x720_45fps.conf**  | 1280x720   | 45  | ~119 MB/s  | Reliable/Volatile | HD intermediate               |
| **1280x720_60fps.conf**  | 1280x720   | 60  | ~158 MB/s  | Reliable/Volatile | HD high framerate             |
| **high.conf**            | 1920x1080  | 30  | ~178 MB/s  | Reliable/Volatile | Full HD standard              |
| **1920x1080_45fps.conf** | 1920x1080  | 45  | ~267 MB/s  | Reliable/Volatile | Full HD intermediate          |
| **1080p60.conf**         | 1920x1080  | 60  | ~356 MB/s  | Reliable/Volatile | Full HD high framerate        |
| **2560x1440_30fps.conf** | 2560x1440  | 30  | ~316 MB/s  | Reliable/Volatile | 2K standard                   |
| **2560x1440_45fps.conf** | 2560x1440  | 45  | ~475 MB/s  | Reliable/Volatile | 2K intermediate               |
| **2560x1440_60fps.conf** | 2560x1440  | 60  | ~633 MB/s  | Reliable/Volatile | 2K high framerate             |
| **extreme.conf**         | 3840x2160  | 30  | ~712 MB/s  | Reliable/Volatile | 4K standard                   |
| **3840x2160_45fps.conf** | 3840x2160  | 45  | ~1068 MB/s | Reliable/Volatile | 4K intermediate ⚠️ Zenoh limit |
| **3840x2160_60fps.conf** | 3840x2160  | 60  | ~1424 MB/s | Reliable/Volatile | 4K maximum ⚠️ Use CycloneDDS   |

#### Alternative QoS Configurations

| Config File                 | Resolution | FPS | Data Rate | QoS                 | Use Case            |
|-----------------------------|------------|-----|-----------|---------------------|---------------------|
| **high_besteffort.conf**    | 1920x1080  | 30  | ~180 MB/s | BestEffort/Volatile | Test lossy behavior |
| **extreme_besteffort.conf** | 3840x2160  | 30  | ~720 MB/s | BestEffort/Volatile | 4K with frame drops |
| **high_transient.conf**     | 1920x1080  | 30  | ~180 MB/s | Reliable/Transient  | Test durability     |

**Note**: Data rates are for uncompressed RGB video (3 bytes per pixel).

⚠️ **Performance Notes** (from `results/run_2025-10-13_00-01-58/`):
- **CycloneDDS**: Consistent 0.00-0.86% frame loss across all loads (13-1424 MB/s), excellent for extreme 4K loads
- **Zenoh**: Perfect 0.00% loss up to 633 MB/s (2K@60fps), 0.04% loss at 4K@45fps (1068 MB/s), degrades at 4K@60fps
- **Recommended**: Zenoh for HD/2K streams, CycloneDDS for 4K@60fps extreme loads

### Using Different Configurations

```bash
# Method 1: Specify config when starting (recommended)
make STRESS_CONFIG=high.conf start-cyclonedds

# Method 2: Set as default (edit Makefile)
# Change: STRESS_CONFIG ?= high.conf

# Method 3: Edit an existing config
make edit-config  # Edits current config (default: medium.conf)

# List all available configs
make list-configs

# View settings of current config
make show-config
```

## Makefile Targets

### Build
```bash
make build              # Build stress test package
make clean              # Clean build artifacts
```

### Benchmark Automation
```bash
make benchmark          # Run complete benchmark suite (14 configs, ~35 min)
make benchmark-quick    # Run quick benchmark (6 configs, ~8 min)
make benchmark-analyze  # Analyze latest benchmark results
```

### Configuration
```bash
make show-config        # Display current configuration
make edit-config        # Edit configuration file
make list-configs       # List all available configurations
```

### CycloneDDS Tests
```bash
make start-cyclonedds   # Start test with CycloneDDS
make stop-cyclonedds    # Stop CycloneDDS test
make status-cyclonedds  # Show service status
make logs-cyclonedds    # View logs
```

### Zenoh Tests
```bash
make start-router       # Start Zenoh router (required first)
make stop-router        # Stop router
make status-router      # Show router status
make logs-router        # View router logs

make start-zenoh        # Start test with Zenoh
make stop-zenoh         # Stop Zenoh test
make status-zenoh       # Show service status
make logs-zenoh         # View logs
```

### Management
```bash
make stop-all           # Stop all services
make status-all         # Show all service statuses
```

## Architecture

```
┌─────────────────────────────────────────────┐
│  Component Container (Multi-threaded)      │
│  ┌────────────────┐    ┌─────────────────┐ │
│  │  gscam_node    │───▶│ Image Subscriber│ │
│  │  (Publisher)   │    │  (Metrics)      │ │
│  └────────────────┘    └─────────────────┘ │
└─────────────────────────────────────────────┘
         │                         │
         └────────RMW Layer────────┘
         (CycloneDDS or Zenoh)
```

Components:
- **gscam_node**: Publishes video using GStreamer (videotestsrc pattern)
- **Image Subscriber**: Receives images, measures latency and throughput
- **Component Container**: Multi-threaded container for efficient IPC

## Performance Metrics

The subscriber node automatically logs performance statistics:

```
[image_subscriber]: Received 300 messages
[image_subscriber]: Latency: avg=2.5ms, min=1.2ms, max=5.8ms
[image_subscriber]: Throughput: 80.2 MB/s (30.0 fps)
```

Monitor with ROS 2 tools:
```bash
# Message rate
ros2 topic hz /camera/image_raw

# Bandwidth
ros2 topic bw /camera/image_raw

# Topic details
ros2 topic info /camera/image_raw -v
```

## Stress Testing Strategies

### Comparing RMW Implementations

Run the same profile with both implementations:

```bash
# Set desired profile
make edit-config  # e.g., PROFILE=high

# Test CycloneDDS
make start-cyclonedds
# Record metrics...
make logs-cyclonedds
make stop-cyclonedds

# Test Zenoh
make start-router
make start-zenoh
# Compare metrics...
make logs-zenoh
make stop-all
```

### Extended Stability Testing

Run for longer periods to detect issues:

```bash
# Start test
make start-cyclonedds

# Monitor for several minutes/hours
watch -n 5 'ros2 topic hz /camera/image_raw'

# Check for:
# - Memory leaks (increasing RSS)
# - CPU degradation
# - Dropped messages
# - Latency increases

# View logs periodically
make logs-cyclonedds
```

## Troubleshooting

### Test won't start
```bash
# Check if package is built
ls install/

# Rebuild if necessary
make clean
make build
```

### No video output
```bash
# Check gscam is installed
ros2 pkg list | grep gscam

# Install if missing
sudo apt install ros-humble-gscam
```

### Node discovery issues
```bash
# Check ROS domain
echo $ROS_DOMAIN_ID

# Use environment scripts
./cyclonedds/cyclonedds_env.sh ros2 node list
./zenoh/zenoh_env.sh ros2 node list
```

### High CPU usage
This is expected with uncompressed video at high resolutions. To reduce:
- Use lower profile
- Enable video compression (future feature)
- Reduce FPS

### Memory issues
Large frame buffers at high resolution require significant memory:
- Each 4K frame is ~24 MB uncompressed
- Ensure sufficient RAM available
- Use lower profile if needed

## Advanced Configuration

See [STRESS_TEST_CONFIG.md](STRESS_TEST_CONFIG.md) for:
- Custom profile configuration
- Video encoding options
- Multiple publishers/subscribers
- GStreamer pipeline customization
- Performance tuning tips

## Dependencies

### Required
- ROS 2 Humble (or later)
- gscam package (`ros-humble-gscam`)
- **ros2systemd** (for service management):
  ```bash
  pip install ros2systemd
  ```
  - Provides `ros2 systemd` command for managing ROS 2 services as systemd units
  - Required for automated benchmark scripts and Makefile targets

### For CycloneDDS Tests
- rmw_cyclonedds_cpp (usually installed with ROS 2)
- **Iceoryx packages** (REQUIRED for shared memory support):
  ```bash
  sudo apt install ros-humble-iceoryx-posh ros-humble-iceoryx-binding-c
  ```
  - These packages provide `/opt/ros/humble/bin/iox-roudi` (RouDi daemon)
  - **Shared memory is REQUIRED** - benchmarks will fail without it
  - Shared memory is enabled in `rmw_config/cyclonedds_shm.xml`
  - RouDi daemon is automatically started/managed by benchmark scripts

### For Zenoh Tests
- rmw_zenoh (built from submodule via `make build-rmw-zenoh` in root)

No Autoware dependency! This test is standalone.

## Shared Memory Configuration

**IMPORTANT**: All benchmarks use shared memory for zero-copy transport. This is required for accurate performance comparisons.

### CycloneDDS (via Iceoryx)
- Configuration: `rmw_config/cyclonedds_shm.xml`
- Requires Iceoryx RouDi daemon (installed via `ros-humble-iceoryx-posh`)
- **Benchmark scripts automatically verify and start RouDi** when needed
- Script will fail with error if Iceoryx is not installed
- Manual control (for non-benchmark usage):
  ```bash
  # Start RouDi daemon
  /opt/ros/humble/bin/iox-roudi &

  # Check if RouDi is running
  pgrep -x iox-roudi

  # Stop RouDi daemon
  killall iox-roudi
  ```

### Zenoh
- Configuration: `rmw_config/zenoh_shm.json5`
- Built-in shared memory support (no external daemon required)
- Shared memory enabled with **256 MB pool** for zero-copy transfers
- Optimized for image streaming from 640x480 to 4K resolution

#### Zenoh Shared Memory Configuration Details

The `rmw_config/zenoh_shm.json5` file is configured for high-throughput benchmarks:

**Key Settings:**
```json5
shared_memory: {
  enabled: true,              // Enable shared memory
  mode: "init",               // Initialize at startup (no first-message latency)
  transport_optimization: {
    pool_size: 268435456,     // 256 MB shared memory pool
    message_size_threshold: 1024,  // Use SHM for messages ≥ 1 KB
  }
}
```

**Pool Size Rationale (256 MB):**
- Test range: 640x480 RGB (900 KB) to 3840x2160 RGB (24 MB)
- Buffer depth: ~10 frames (history_depth=10 + TX/RX queues)
- Required: 24 MB × 10 = 240 MB minimum
- Configured: 256 MB (power of 2, provides ~10-11 4K frames)

**Comparison with CycloneDDS:**
| Aspect | Zenoh | CycloneDDS (Iceoryx) |
|--------|-------|----------------------|
| Pool type | Single unified pool | 5 tiered pools (1KB-25MB) |
| Total size | 256 MB | 1.4 GB |
| Threshold | 1 KB (all images use SHM) | Variable per pool |
| Daemon | None (built-in) | RouDi daemon required |
| Init mode | "init" (immediate) | Daemon must pre-start |

**Alternative Pool Sizes:**
- **128 MB**: Memory constrained systems, sufficient for ≤1080p@60fps
- **512 MB**: Multiple simultaneous high-res streams
- **1 GB**: Match CycloneDDS-level buffering for extreme cases

**Transport Optimizations:**
- 16 MB RX/TX buffers (vs default 64 KB) for large image bursts
- QoS enabled with 8 priority levels (required for ROS 2)
- Compression disabled (CPU overhead not worth it for local SHM)
- TCP transport on port 7448 (avoids conflict with planning sim router)

**Message Size Threshold (1 KB):**
- Default Zenoh threshold is 3 KB
- Lowered to 1 KB to catch more messages in shared memory
- All image data (smallest: 900 KB) uses zero-copy SHM transport
- Small ROS messages (< 1 KB) use regular TCP transport

## Files

```
gscam_stress/
├── Makefile                          # Build and run targets
├── README.md                         # This file
├── BENCHMARK_FINDINGS.md             # Comprehensive benchmark analysis
├── STRESS_TEST_CONFIG.md             # Detailed configuration guide
├── scripts/                          # Benchmark automation scripts
│   ├── README.md                     # Scripts documentation
│   ├── benchmark_rmw.sh              # Single test runner
│   ├── run_boundary_test.sh          # Full benchmark suite (14 configs)
│   ├── quick_boundary_test.sh        # Quick test (6 configs)
│   └── analyze_results.py            # Results analysis tool
├── results/                          # Benchmark results (generated)
│   ├── run_TIMESTAMP/                # Full benchmark run
│   │   ├── summary.csv               # Aggregate results
│   │   ├── findings.txt              # Analysis report
│   │   ├── cyclonedds/               # CycloneDDS CSV files
│   │   └── zenoh/                    # Zenoh CSV files
│   └── quick_run_TIMESTAMP/          # Quick benchmark run
├── src/stress_test/
│   ├── config/                       # Stress test configurations
│   │   ├── low.conf                  # 640x480 @ 15 FPS
│   │   ├── 640x480_30fps.conf        # 640x480 @ 30 FPS
│   │   ├── medium.conf               # 1280x720 @ 30 FPS (default)
│   │   ├── 1280x720_45fps.conf       # 1280x720 @ 45 FPS
│   │   ├── 1280x720_60fps.conf       # 1280x720 @ 60 FPS
│   │   ├── high.conf                 # 1920x1080 @ 30 FPS
│   │   ├── 1920x1080_45fps.conf      # 1920x1080 @ 45 FPS
│   │   ├── 1080p60.conf              # 1920x1080 @ 60 FPS
│   │   ├── 2560x1440_30fps.conf      # 2560x1440 @ 30 FPS
│   │   ├── 2560x1440_45fps.conf      # 2560x1440 @ 45 FPS
│   │   ├── 2560x1440_60fps.conf      # 2560x1440 @ 60 FPS
│   │   ├── extreme.conf              # 3840x2160 @ 30 FPS
│   │   ├── 3840x2160_45fps.conf      # 3840x2160 @ 45 FPS
│   │   ├── 3840x2160_60fps.conf      # 3840x2160 @ 60 FPS
│   │   ├── high_besteffort.conf      # Full HD with BEST_EFFORT QoS
│   │   ├── extreme_besteffort.conf   # 4K with BEST_EFFORT QoS
│   │   ├── high_transient.conf       # Full HD with TRANSIENT_LOCAL
│   │   ├── custom.conf.example       # Template for custom configs
│   │   └── camera.conf.example       # Template for real cameras
│   ├── launch/
│   │   └── stress_test.launch.py     # Launch file
│   └── src/
│       └── image_subscriber_node.cpp # Subscriber with metrics
├── rmw_config/                       # RMW implementation configs
│   ├── cyclonedds_shm.xml            # CycloneDDS with Iceoryx shared memory
│   └── zenoh_shm.json5               # Zenoh with shared memory
├── cyclonedds/
│   ├── cyclonedds_env.sh             # CycloneDDS environment
│   └── *.csv                         # CycloneDDS test results (generated)
└── zenoh/
    ├── zenoh_env.sh                  # Zenoh environment
    └── *.csv                         # Zenoh test results (generated)
```

## License

Same as parent project.

## Contributing

To add new features:
1. Update configuration file with new parameters
2. Modify launch file to use parameters
3. Update documentation
4. Test with all profiles

## See Also

- [results/run_2025-10-13_00-01-58/summary.csv](results/run_2025-10-13_00-01-58/summary.csv) - Benchmark results
- [scripts/README.md](scripts/README.md) - Benchmark automation documentation
- [Parent README](../README.md)
- [Stress Test Configuration Guide](STRESS_TEST_CONFIG.md)
