# gscam Stress Test for RMW Implementations

A configurable stress test for comparing ROS 2 RMW implementations (CycloneDDS vs Zenoh) using high-resolution video streaming.

## Quick Start

**Recommended approach**: Start by building and running the automated benchmark suite to get comprehensive performance comparison data.

### Configuration Files

Before running tests, familiarize yourself with the configuration locations:

- **RMW configurations** (shared memory settings): `rmw_config/`
  - `rmw_config/cyclonedds_shm.xml` - CycloneDDS with Iceoryx shared memory
  - `rmw_config/zenoh_shm.json5` - Zenoh with shared memory (1.4 GB pool)
- **Test configurations** (resolution, FPS, QoS): `src/stress_test/config/`
  - `src/stress_test/config/medium.conf` - Default: 1280x720@30fps
  - `src/stress_test/config/high.conf` - 1920x1080@30fps
  - `src/stress_test/config/extreme.conf` - 3840x2160@30fps
  - See [Configuration Profiles](#configuration-profiles) for all 14 configs

### 1. Build and Run Automated Benchmarks (Recommended)

```bash
# Build the stress test package
make build

# Run quick benchmark (6 configs, ~8 minutes)
# Tests both CycloneDDS and Zenoh across key configurations
make benchmark-quick

# View results
make benchmark-analyze
cat results/quick_run_*/summary.csv

# Optional: Run full benchmark suite (14 configs, ~35 minutes)
make benchmark
```

The automated benchmark suite will:
- Test both RMW implementations (CycloneDDS and Zenoh)
- Run through multiple configurations automatically
- Generate CSV results and analysis reports
- Provide comprehensive performance comparison data

All benchmark scripts are in `scripts/` directory. See [scripts/README.md](scripts/README.md) for details.

### 2. Manual Testing (Advanced)

For manual control and experimentation with specific configurations:

```bash
# View current configuration (default: medium.conf)
make show-config

# List available configurations
make list-configs

# Test with CycloneDDS
make start-cyclonedds-router      # Start Iceoryx RouDi (required for shared memory)
make start-cyclonedds-test        # Start stress test with default config

# Monitor performance in another terminal
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw

# Stop test
make stop-cyclonedds-test
make stop-cyclonedds-router

# Test with different config
make STRESS_CONFIG=high.conf start-cyclonedds-test
# Monitor...
make stop-cyclonedds-test

# Compare with Zenoh
make start-zenoh-router           # Start Zenoh router (required)
make STRESS_CONFIG=high.conf start-zenoh-test
# Monitor and compare...
make stop-all                     # Stop all services
```

## Benchmark Results Summary

**Latest benchmark run**: `results/run_2025-10-22_14-40-25/` (with Zenoh 1.4 GB pool)

### Performance Comparison: CycloneDDS vs Zenoh

Comprehensive testing across 14 configurations (13 MB/s to 1424 MB/s) with Reliable/Volatile QoS and **shared memory enabled** for both implementations:

| Configuration   | Resolution | FPS | Data Rate | CycloneDDS Loss % | Zenoh Loss % | CycloneDDS Latency (ms) | Zenoh Latency (ms) | Winner                  |
|-----------------|------------|-----|-----------|-------------------|--------------|-------------------------|--------------------|-------------------------|
| low             | 640x480    | 15  | 13 MB/s   | 0.00%             | 0.00%        | 0.62                    | 0.79               | Tie (both excellent)    |
| 640x480_30fps   | 640x480    | 30  | 26 MB/s   | 0.00%             | 0.00%        | 0.61                    | 0.77               | Tie (both excellent)    |
| medium          | 1280x720   | 30  | 79 MB/s   | 0.00%             | 0.00%        | 1.65                    | 2.57               | Tie (CycloneDDS faster) |
| 1280x720_45fps  | 1280x720   | 45  | 119 MB/s  | 0.00%             | 0.00%        | 1.64                    | 2.54               | Tie (CycloneDDS faster) |
| 1280x720_60fps  | 1280x720   | 60  | 158 MB/s  | 0.00%             | 0.00%        | 1.62                    | 2.60               | Tie (CycloneDDS faster) |
| high            | 1920x1080  | 30  | 178 MB/s  | 0.00%             | 0.00%        | 5.10                    | 4.34               | Tie (Zenoh faster)      |
| 1920x1080_45fps | 1920x1080  | 45  | 267 MB/s  | 0.00%             | 0.00%        | 5.00                    | 4.46               | Tie (Zenoh faster)      |
| 1080p60         | 1920x1080  | 60  | 356 MB/s  | 0.00%             | 0.00%        | 5.01                    | 4.44               | Tie (Zenoh faster)      |
| 2560x1440_30fps | 2560x1440  | 30  | 316 MB/s  | 0.00%             | 0.00%        | 6.37                    | 8.81               | Tie (CycloneDDS faster) |
| 2560x1440_45fps | 2560x1440  | 45  | 475 MB/s  | 0.00%             | 0.04%        | 6.02                    | 8.95               | Tie (both excellent)    |
| 2560x1440_60fps | 2560x1440  | 60  | 633 MB/s  | 0.00%             | 0.03%        | 6.75                    | 8.86               | Tie (both excellent)    |
| extreme         | 3840x2160  | 30  | 712 MB/s  | 0.00%             | 0.05%        | 13.18                   | 28.27              | Tie (both excellent)    |
| 3840x2160_45fps | 3840x2160  | 45  | 1068 MB/s | -0.00%            | 0.04%        | 13.04                   | 31.75              | Tie (both excellent)    |
| 3840x2160_60fps | 3840x2160  | 60  | 1424 MB/s | 0.06%             | 36.75%       | 13.19                   | 34.88              | **CycloneDDS**          |

### Key Findings

**Maximum Capability Analysis:**

1. **CycloneDDS (with Iceoryx shared memory - 1.4 GB)**
   - **Perfect 0.00% frame loss** across nearly all data rates (13-1068 MB/s)
   - Minor degradation only at extreme 4K@60fps: 0.06% loss at 1424 MB/s
   - Low, stable latency: 0.63ms @ 26 MB/s, 13.19ms @ 1424 MB/s
   - **Max tested capability**: 1424 MB/s (4K@60fps) with 0.06% frame loss
   - Configuration: 1.4 GB Iceoryx memory pools (5 tiers: 1KB-25MB)

2. **Zenoh (with 1.4 GB shared memory pool)**
   - **Perfect 0.00-0.04% frame loss up to 1068 MB/s (4K@45fps)**
   - **Severe degradation at 4K@60fps: 36.75% loss at 1424 MB/s**
   - Higher latency than CycloneDDS at 4K loads: 31.75ms @ 1068 MB/s, 34.88ms @ 1424 MB/s
   - **Max tested capability**: 1068 MB/s (4K@45fps) with <0.1% frame loss
   - Configuration: 1.4 GB unified shared memory pool (matches CycloneDDS total)

**Pool Size Paradox (Zenoh):**

⚠️ **Critical finding**: Larger Zenoh pools perform WORSE at extreme loads!

Comparison of 256 MB vs 512 MB vs 1.4 GB pool configurations:

| Pool Size | 4K@30fps (712 MB/s) | 4K@45fps (1068 MB/s) | 4K@60fps (1424 MB/s) |
|-----------|---------------------|----------------------|----------------------|
| 256 MB    | 0.00% ✅            | 6.09% ❌             | 17.69% ❌            |
| 512 MB    | 0.05% ✅            | 1.64% ⚠️             | 22.62% ❌            |
| 1.4 GB    | 0.00% ✅            | 0.04% ✅             | **36.75%** ⛔       |

**Analysis:**
- 1.4 GB pool provides BEST performance at 4K@45fps (0.04% loss)
- But provides WORST performance at 4K@60fps (36.75% loss - 600x worse than CycloneDDS!)
- This suggests Zenoh's unified pool has memory management overhead that scales poorly
- CycloneDDS's tiered pool architecture (5 pools: 1KB-25MB) is fundamentally more efficient
- **Conclusion**: Pool size alone doesn't determine performance - architecture matters more

**Reliability Analysis:**

- **Low to High Load (13-633 MB/s)**:
  - **Both implementations**: Perfect 0.00-0.04% frame loss
  - CycloneDDS: Lower latency at HD/2K resolutions (1-7ms)
  - Zenoh: Lower latency at Full HD resolutions (4ms vs 5ms)
  - **Result**: Tie - both excellent for production use

- **Extreme Load 4K@30fps (712 MB/s)**:
  - CycloneDDS: 0.00% loss, 13.18ms latency
  - Zenoh: 0.00% loss, 27.43ms latency
  - **Result**: Tie - both handle 4K@30fps excellently (CycloneDDS has lower latency)

- **Extreme Load 4K@45fps (1068 MB/s)**:
  - CycloneDDS: -0.00% loss, 13.04ms latency (excellent)
  - Zenoh: 0.04% loss, 31.75ms latency (excellent)
  - **Result**: Tie - both excellent (CycloneDDS has lower latency)

- **Extreme Load 4K@60fps (1424 MB/s)**:
  - CycloneDDS: 0.06% loss, 13.19ms latency (excellent)
  - Zenoh: **36.75% loss**, 34.88ms latency (unusable - pool size paradox)
  - **Result**: CycloneDDS clearly superior (600x better frame loss)

**Recommendations:**

- **For HD/Full HD/2K streaming (up to 633 MB/s)**: Either implementation works perfectly
- **For 4K@30fps (712 MB/s)**: Either implementation works excellently
- **For 4K@45fps (1068 MB/s)**: **Either implementation works excellently** (both <0.1% loss)
- **For 4K@60fps (1424 MB/s)**: **Use CycloneDDS** - Zenoh suffers from pool size paradox (36.75% loss)
- **For simplicity (no daemon required)**: Zenoh is excellent up to 4K@45fps
- **For maximum reliability across all loads**: CycloneDDS with Iceoryx (tiered pool architecture)

**Configuration Notes:**

- CycloneDDS uses 1.4 GB Iceoryx shared memory (5 tiered pools) - see `cyclonedds/roudi_config.toml`
- Zenoh uses 1.4 GB unified pool (matches CycloneDDS total) - see `rmw_config/zenoh_shm.json5`
- Pool size evolution: 256 MB → 512 MB → 1.4 GB
  - Larger pools improved 4K@45fps (6.09% → 1.64% → 0.04%)
  - But worsened 4K@60fps (17.69% → 22.62% → **36.75%**) - pool size paradox!
- **Key insight**: Architecture (tiered vs unified) matters more than pool size alone

See `results/run_2025-10-22_14-40-25/summary.csv` for complete raw data.

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

⚠️ **Performance Notes** (from `results/run_2025-10-22_14-40-25/`):
- **CycloneDDS**: Consistent 0.00-0.86% frame loss across all loads (13-1424 MB/s), excellent for extreme 4K loads
- **Zenoh**: Perfect 0.00% loss up to 633 MB/s (2K@60fps), 0.04% loss at 4K@45fps (1068 MB/s), degrades at 4K@60fps
- **Recommended**: Zenoh for HD/2K streams, CycloneDDS for 4K@60fps extreme loads

### Using Different Configurations

```bash
# Method 1: Specify config when starting (recommended)
make STRESS_CONFIG=high.conf start-cyclonedds-test

# Method 2: Set as default (edit Makefile)
# Change: STRESS_CONFIG ?= high.conf

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

### Benchmark Automation (Recommended)
```bash
make benchmark          # Run complete benchmark suite (14 configs, ~35 min)
make benchmark-quick    # Run quick benchmark (6 configs, ~8 min)
make benchmark-analyze  # Analyze latest benchmark results
```

### Configuration
```bash
make show-config        # Display current configuration
make list-configs       # List all available configurations
```

### CycloneDDS Tests
```bash
# CycloneDDS Router (Iceoryx RouDi)
make start-cyclonedds-router   # Start Iceoryx RouDi daemon (required for shared memory)
make stop-cyclonedds-router    # Stop Iceoryx RouDi daemon
make status-cyclonedds-router  # Show RouDi daemon status
make logs-cyclonedds-router    # View RouDi daemon logs

# CycloneDDS Stress Test
make start-cyclonedds-test     # Start stress test with CycloneDDS
make stop-cyclonedds-test      # Stop CycloneDDS test
make status-cyclonedds-test    # Show service status
make logs-cyclonedds-test      # View logs
```

### Zenoh Tests
```bash
# Zenoh Router
make start-zenoh-router        # Start Zenoh router (required first)
make stop-zenoh-router         # Stop router
make status-zenoh-router       # Show router status
make logs-zenoh-router         # View router logs

# Zenoh Stress Test
make start-zenoh-test          # Start stress test with Zenoh
make stop-zenoh-test           # Stop Zenoh test
make status-zenoh-test         # Show service status
make logs-zenoh-test           # View logs
```

### Management
```bash
make stop-all           # Stop all services (both RMWs and routers)
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

**Recommended**: Use the automated benchmark suite (`make benchmark-quick` or `make benchmark`) for comprehensive comparison.

For manual comparison with specific profiles:

```bash
# Test CycloneDDS
make start-cyclonedds-router
make STRESS_CONFIG=high.conf start-cyclonedds-test
# Record metrics...
make logs-cyclonedds-test
make stop-cyclonedds-test
make stop-cyclonedds-router

# Test Zenoh
make start-zenoh-router
make STRESS_CONFIG=high.conf start-zenoh-test
# Compare metrics...
make logs-zenoh-test
make stop-all
```

### Extended Stability Testing

Run for longer periods to detect issues:

```bash
# Start test
make start-cyclonedds-router
make start-cyclonedds-test

# Monitor for several minutes/hours
watch -n 5 'ros2 topic hz /camera/image_raw'

# Check for:
# - Memory leaks (increasing RSS)
# - CPU degradation
# - Dropped messages
# - Latency increases

# View logs periodically
make logs-cyclonedds-test
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
- Shared memory enabled with **1.4 GB pool** (matches CycloneDDS total)
- Optimized for image streaming from 640x480 to 4K@45fps

#### Zenoh Shared Memory Configuration Details

The `rmw_config/zenoh_shm.json5` file is configured for high-throughput benchmarks:

**Key Settings:**
```json5
shared_memory: {
  enabled: true,              // Enable shared memory
  mode: "init",               // Initialize at startup (no first-message latency)
  transport_optimization: {
    pool_size: 1500000000,    // 1.4 GB shared memory pool (matches CycloneDDS)
    message_size_threshold: 1024,  // Use SHM for messages ≥ 1 KB
  }
}
```

**Pool Size Rationale (1.4 GB):**
- Matches CycloneDDS/Iceoryx total pool size for fair comparison
- CycloneDDS uses 5 tiered pools totaling 1,452,478,800 bytes
- This config uses single unified pool of 1,500,000,000 bytes (1.4 GB)
- **Performance**: Excellent up to 4K@45fps (0.04% loss), but suffers pool size paradox at 4K@60fps (36.75% loss)

**Pool Size Paradox (Benchmark Results):**
- **256 MB**: Excellent 4K@30fps, 6.09% loss @ 4K@45fps, 17.69% loss @ 4K@60fps
- **512 MB**: Excellent 4K@30fps, 1.64% loss @ 4K@45fps, 22.62% loss @ 4K@60fps
- **1.4 GB** (current): Excellent 4K@45fps (0.04% loss), **36.75% loss @ 4K@60fps** (WORST!)
- **Conclusion**: Larger unified pools have memory management overhead that degrades performance at extreme loads

**Comparison with CycloneDDS:**
| Aspect | Zenoh | CycloneDDS (Iceoryx) |
|--------|-------|----------------------|
| Pool type | Single unified pool | 5 tiered pools (1KB-25MB) |
| Total size | 1.4 GB | 1.4 GB |
| Threshold | 1 KB (all images use SHM) | Variable per pool |
| Daemon | None (built-in) | RouDi daemon required |
| Init mode | "init" (immediate) | Daemon must pre-start |
| Max reliable rate | ~1068 MB/s (4K@45fps, <0.1% loss) | ~1424 MB/s (4K@60fps, 0.06% loss) |
| Architecture advantage | Simpler setup | Better at extreme loads |

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
├── scripts/                          # Benchmark automation scripts
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
│   │   └── ...
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

- [results/run_2025-10-22_14-40-25/summary.csv](results/run_2025-10-22_14-40-25/summary.csv) - Benchmark results
- [scripts/README.md](scripts/README.md) - Benchmark automation documentation
- [Parent README](../README.md)
- [Stress Test Configuration Guide](STRESS_TEST_CONFIG.md)
