# gscam Stress Test for RMW Implementations

A configurable stress test for comparing ROS 2 RMW implementations (CycloneDDS vs Zenoh) using high-resolution video streaming.

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

| Config File | Resolution | FPS | Data Rate | QoS | Use Case |
|------------|-----------|-----|-----------|-----|----------|
| **low.conf** | 640x480 | 15 | ~14 MB/s | Reliable/Volatile | Initial testing, debugging |
| **medium.conf** | 1280x720 | 30 | ~80 MB/s | Reliable/Volatile | Standard HD (default) |
| **high.conf** | 1920x1080 | 30 | ~180 MB/s | Reliable/Volatile | Full HD stress test |
| **extreme.conf** | 3840x2160 | 30 | ~720 MB/s | Reliable/Volatile | 4K maximum stress |
| **high_besteffort.conf** | 1920x1080 | 30 | ~180 MB/s | BestEffort/Volatile | Test lossy behavior |
| **extreme_besteffort.conf** | 3840x2160 | 30 | ~720 MB/s | BestEffort/Volatile | 4K with frame drops |
| **high_transient.conf** | 1920x1080 | 30 | ~180 MB/s | Reliable/Transient | Test durability |
| **1080p60.conf** | 1920x1080 | 60 | ~360 MB/s | Reliable/Volatile | High FPS variant |

**Note**: Data rates are for uncompressed RGB video.

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

### Configuration
```bash
make show-config        # Display current configuration
make edit-config        # Edit configuration file
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

## System Requirements

### Minimum (low profile)
- CPU: 2 cores
- RAM: 2 GB
- Bandwidth: 15 MB/s

### Recommended (medium/high profile)
- CPU: 4 cores
- RAM: 4 GB
- Bandwidth: 200 MB/s

### High Performance (extreme profile)
- CPU: 8+ cores
- RAM: 8 GB
- Bandwidth: 1 GB/s
- Note: 4K @ 30fps may exceed 1 Gbps Ethernet without shared memory

## Stress Testing Strategies

### Progressive Load Testing

Start with low stress and gradually increase:

```bash
# 1. Verify setup with LOW
sed -i 's/PROFILE=.*/PROFILE=low/' src/stress_test/config/stress_test.conf
make start-cyclonedds
# Verify no errors, good baseline
make stop-cyclonedds

# 2. Standard load with MEDIUM
sed -i 's/PROFILE=.*/PROFILE=medium/' src/stress_test/config/stress_test.conf
make start-cyclonedds
# Monitor CPU, memory, latency
make stop-cyclonedds

# 3. High stress with HIGH
sed -i 's/PROFILE=.*/PROFILE=high/' src/stress_test/config/stress_test.conf
make start-cyclonedds
# Look for dropped frames, increased latency
make stop-cyclonedds

# 4. Maximum stress with EXTREME
sed -i 's/PROFILE=.*/PROFILE=extreme/' src/stress_test/config/stress_test.conf
make start-cyclonedds
# Expect high resource usage, find limits
make stop-cyclonedds
```

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

### For CycloneDDS Tests
- rmw_cyclonedds_cpp (usually installed with ROS 2)

### For Zenoh Tests
- rmw_zenoh (built from submodule via `make build-rmw-zenoh` in root)

No Autoware dependency! This test is standalone.

## Files

```
gscam_stress/
├── Makefile                          # Build and run targets
├── README.md                         # This file
├── STRESS_TEST_CONFIG.md             # Detailed configuration guide
├── src/stress_test/
│   ├── config/                       # Stress test configurations
│   │   ├── low.conf                  # 640x480 @ 15 FPS
│   │   ├── medium.conf               # 1280x720 @ 30 FPS (default)
│   │   ├── high.conf                 # 1920x1080 @ 30 FPS
│   │   ├── extreme.conf              # 3840x2160 @ 30 FPS
│   │   ├── high_besteffort.conf      # Full HD with BEST_EFFORT QoS
│   │   ├── extreme_besteffort.conf   # 4K with BEST_EFFORT QoS
│   │   ├── high_transient.conf       # Full HD with TRANSIENT_LOCAL
│   │   ├── 1080p60.conf              # 1920x1080 @ 60 FPS
│   │   ├── custom.conf.example       # Template for custom configs
│   │   └── camera.conf.example       # Template for real cameras
│   ├── rmw_config/                   # RMW implementation configs
│   │   ├── cyclonedds_shm.xml        # CycloneDDS configuration
│   │   └── zenoh_shm.json5           # Zenoh configuration
│   ├── launch/
│   │   └── stress_test.launch.py     # Launch file
│   └── src/
│       └── image_subscriber_node.cpp # Subscriber with metrics
├── cyclonedds/
│   └── cyclonedds_env.sh            # CycloneDDS environment
└── zenoh/
    └── zenoh_env.sh                 # Zenoh environment
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

- [Parent README](../README.md)
- [Stress Test Configuration Guide](STRESS_TEST_CONFIG.md)
- [Dependencies Overview](../DEPENDENCIES.md)
- [Setup Guide](../SETUP.md)
