# RMW Stress Test: CycloneDDS vs Zenoh with Shared Memory

This stress test compares CycloneDDS and Zenoh middleware performance using high-resolution video streaming with **shared memory transport** enabled.

## Overview

The stress test consists of:
- **gscam publisher**: Generates high-resolution video frames (default: 1920x1080 @ 30 FPS)
- **Image subscriber**: Measures latency, throughput, and frame rate
- **Component container**: Both nodes run as composable nodes for efficient intra-process communication
- **Shared memory**: Both middlewares configured to use shared memory for zero-copy transport

## Architecture

```
┌─────────────────────────────────────────┐
│  Component Container (MT)               │
│  ┌───────────────┐  ┌────────────────┐ │
│  │  gscam Node   │  │ Image          │ │
│  │  (Publisher)  │→→│ Subscriber     │ │
│  │               │  │ (Statistics)   │ │
│  └───────────────┘  └────────────────┘ │
└─────────────────────────────────────────┘
           ↓
    Shared Memory Transport
  (Zero-copy for large messages)
```

## Shared Memory Configuration

### CycloneDDS (via Iceoryx)

CycloneDDS uses **Iceoryx** for shared memory transport:

**File**: `config/cyclonedds_shm.xml`

Key features:
- `<SharedMemory><Enable>true</Enable>`: Enables shared memory
- `<Iceoryx><Enable>true</Enable>`: Uses Iceoryx middleware
- Zero-copy transport for messages > threshold size
- Optimized for intra-host communication

**Environment variable**:
```bash
export CYCLONEDDS_URI=file://$(pwd)/config/cyclonedds_shm.xml
```

### Zenoh

Zenoh has **built-in shared memory** support:

**File**: `config/zenoh_shm.json5`

Key features:
- `shared_memory.enabled: true`: Enables SHM transport
- Native zero-copy support
- Automatic fallback to network transport if needed

**Environment variable**:
```bash
export ZENOH_CONFIG=file://$(pwd)/config/zenoh_shm.json5
```

## Quick Start

### Prerequisites

```bash
# Install gscam (if not already installed)
sudo apt install ros-humble-gscam

# Build the benchmark workspace (includes stress test package)
cd /home/aeon/repos/autoware/2025.02-ws/autoware-rmw-zenoh-benchmark
make build
```

### Running Tests

All stress tests are managed via systemd services for clean process management and easy monitoring.

#### Test with CycloneDDS

```bash
# Start the stress test
make start-stress-cyclonedds

# View logs in real-time
make logs-stress-cyclonedds

# Check status
make status-stress-cyclonedds

# Stop the test
make stop-stress-cyclonedds
```

#### Test with Zenoh

```bash
# Step 1: Start Zenoh router
make start-stress-zenoh-router

# Step 2: Start the stress test
make start-stress-zenoh

# View logs in real-time
make logs-stress-zenoh

# Check status
make status-stress-zenoh
make status-stress-zenoh-router

# Stop the test
make stop-stress-zenoh
make stop-stress-zenoh-router
```

#### Stop All Stress Tests

```bash
# Stop all stress test services at once
make stop-stress-all
```

### Custom Resolution/FPS

Edit the Makefile variables:

```makefile
STRESS_WIDTH := 3840    # 4K resolution
STRESS_HEIGHT := 2160
STRESS_FPS := 60        # High frame rate
```

The systemd services automatically handle environment configuration. Custom parameters are set via Makefile variables (see above).

## Performance Metrics

The image subscriber reports these metrics every 2 seconds:

```
=== Image Subscriber Statistics ===
  Resolution: 1920x1080 (rgb8)
  Frames: 150
  FPS: 30.12
  Bandwidth: 1486.32 Mbps
  Latency - Avg: 12.45 ms, Min: 8.23 ms, Max: 25.67 ms
  Runtime: 5.00 seconds
```

### Metrics Explained

- **Resolution**: Image dimensions and encoding
- **Frames**: Total frames received
- **FPS**: Actual frame rate (should match configured FPS)
- **Bandwidth**: Data throughput in megabits per second
- **Latency**: End-to-end latency (publisher timestamp → subscriber receipt)
  - **Avg**: Average latency across all frames
  - **Min**: Best-case latency
  - **Max**: Worst-case latency

## Expected Results

### With Shared Memory

**1920x1080 @ 30 FPS:**
- **Bandwidth**: ~1.5 Gbps (60 MB/s uncompressed RGB)
- **Latency**: 5-15 ms typical
- **CPU usage**: Low (zero-copy reduces overhead)

**3840x2160 @ 60 FPS (4K):**
- **Bandwidth**: ~12 Gbps (1.5 GB/s uncompressed RGB)
- **Latency**: 10-30 ms typical
- **CPU usage**: Moderate (high frame rate)

### Without Shared Memory

Without shared memory, you would see:
- **Higher latency**: 2-5x increase due to memory copies
- **Higher CPU usage**: Significant overhead from copying large buffers
- **Potential frame drops**: At high resolutions/FPS

## Verifying Shared Memory Usage

### CycloneDDS (Iceoryx)

Check if Iceoryx is running:
```bash
# Check for Iceoryx shared memory segments
ls -lh /dev/shm/ice*

# Monitor Iceoryx logs
export CYCLONEDDS_VERBOSITY=trace
# Look for "SharedMemory" or "Iceoryx" in output
```

### Zenoh

Check Zenoh shared memory:
```bash
# Check for Zenoh SHM segments
ls -lh /dev/shm/zenoh*

# Enable Zenoh debug logs
export RUST_LOG=zenoh=debug
# Look for "shm" or "shared_memory" in output
```

### System Monitoring

Monitor shared memory usage:
```bash
# View all shared memory segments
ipcs -m

# Monitor memory usage
watch -n 1 free -h
```

## Comparison Methodology

### Test Procedure

1. **Clean environment**
   ```bash
   make stop-stress-all
   make stop-all
   ```

2. **Run CycloneDDS test** (5 minutes)
   ```bash
   # Start the test
   make start-stress-cyclonedds

   # Monitor logs in real-time
   make logs-stress-cyclonedds

   # Let run for 5 minutes, note statistics
   ```

3. **Stop and clean**
   ```bash
   make stop-stress-cyclonedds
   sleep 5
   ```

4. **Run Zenoh test** (5 minutes)
   ```bash
   # Start router and test
   make start-stress-zenoh-router
   make start-stress-zenoh

   # Monitor logs in real-time
   make logs-stress-zenoh

   # Let run for 5 minutes, note statistics
   ```

5. **Compare results**
   - Average latency
   - Max latency (jitter)
   - Frame drops (if any)
   - CPU usage (`htop` or `top`)

6. **View service status**
   ```bash
   make status-all
   ```

### Recommended Test Configurations

| Resolution | FPS | Expected Bandwidth | Use Case |
|------------|-----|-------------------|----------|
| 640x480    | 30  | ~220 Mbps         | Baseline test |
| 1920x1080  | 30  | ~1.5 Gbps         | **Default test** |
| 1920x1080  | 60  | ~3.0 Gbps         | High FPS stress |
| 3840x2160  | 30  | ~6.0 Gbps         | 4K resolution |
| 3840x2160  | 60  | ~12 Gbps          | **Extreme stress** |

## Troubleshooting

### Service not starting or crashes

All stress tests run as systemd user services. Use these commands to diagnose issues:

```bash
# Check service status
make status-stress-cyclonedds
make status-stress-zenoh
make status-stress-zenoh-router

# View logs (follows in real-time)
make logs-stress-cyclonedds
make logs-stress-zenoh
make logs-stress-zenoh-router

# Check all service statuses at once
make status-all

# Stop and restart a service
make stop-stress-cyclonedds
make start-stress-cyclonedds

# View systemd journal directly
journalctl --user -u ros2-stress-test-cyclonedds.service
journalctl --user -u ros2-stress-test-zenoh.service
journalctl --user -u ros2-stress-test-zenoh-router.service
```

### gscam not found

```bash
sudo apt install ros-humble-gscam
```

### Iceoryx not working (CycloneDDS)

CycloneDDS may not have Iceoryx support enabled. Check:

```bash
# Fallback: CycloneDDS without Iceoryx (still uses shared memory via POSIX SHM)
# The configuration will still optimize for local transport
```

### Zenoh router not starting

```bash
# Check router status
make status-stress-zenoh-router

# View router logs for errors
make logs-stress-zenoh-router

# Check if rmw_zenohd is available
which rmw_zenohd

# Restart the router
make stop-stress-zenoh-router
make start-stress-zenoh-router
```

### High latency observed

Check CPU throttling:
```bash
# Disable CPU frequency scaling
sudo cpupower frequency-set -g performance
```

### Shared memory segments not created

```bash
# Check /dev/shm permissions
ls -ld /dev/shm

# Should show: drwxrwxrwt
# If not, check system tmpfs configuration
```

### Low FPS / Frame drops

Possible causes:
1. **System load**: Close other applications
2. **CPU throttling**: Check `cpupower frequency-info`
3. **USB camera bandwidth**: Use videotestsrc instead
4. **QoS mismatch**: Check publisher/subscriber QoS settings

## Files

```
autoware-rmw-zenoh-benchmark/
├── Makefile                    # Build and test automation
├── src/
│   └── stress_test/
│       ├── CMakeLists.txt              # Build configuration
│       ├── package.xml                 # ROS package metadata
│       ├── README.md                   # This file
│       ├── config/
│       │   ├── cyclonedds_shm.xml     # CycloneDDS shared memory config
│       │   └── zenoh_shm.json5        # Zenoh shared memory config
│       ├── launch/
│       │   └── stress_test.launch.py  # Launch file
│       └── src/
│           └── image_subscriber_node.cpp  # Subscriber with statistics
├── build/                      # Colcon build output
└── install/                    # Install directory with setup.bash
```

## References

- **CycloneDDS Shared Memory**: https://github.com/eclipse-cyclonedds/cyclonedds
- **Iceoryx**: https://github.com/eclipse-iceoryx/iceoryx
- **Zenoh Shared Memory**: https://zenoh.io/docs/manual/abstractions/#shared-memory
- **gscam**: https://github.com/ros-drivers/gscam

---
*Last Updated: 2025-10-11*
