# Autoware RMW Zenoh Benchmark

Comprehensive benchmarking tools for comparing ROS 2 RMW implementations (CycloneDDS vs Zenoh) with high-bandwidth sensor streaming tests.

## Prerequisites

### Quick Installation (Recommended)

Use the provided Makefile to install all dependencies:

```bash
# Install all dependencies
make install-deps

# Or install selectively
make install-ros-deps      # ROS 2 packages only
make install-python-deps   # Python packages only

# Verify installation
make verify-deps
```

### Manual Installation

If you prefer to install manually:

```bash
# RMW implementations
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-zenoh-cpp

# Iceoryx for CycloneDDS shared memory support
sudo apt install ros-humble-iceoryx-posh ros-humble-iceoryx-binding-c

# Systemd service management for ROS 2
pip3 install --user ros2systemd

# Build tools
sudo apt install python3-colcon-common-extensions
```

### Verify Installation

```bash
# Using Makefile (recommended)
make verify-deps

# Or manually
ros2 systemd --help
ros2 pkg list | grep rmw
```

## Experiments

This repository contains two benchmarking experiments for comparing ROS 2 RMW implementations:

### 1. gscam_stress - High-Bandwidth Sensor Streaming Benchmark (Recommended)

Automated benchmark suite comparing CycloneDDS vs Zenoh for high-bandwidth image streaming.

**Features:**
- 14 test configurations from 13 MB/s to 1424 MB/s (640x480@15fps to 4K@60fps)
- Automated test execution and analysis
- Performance metrics: frame loss, latency, throughput
- Latest results: `gscam_stress/results/run_2025-10-22_14-40-25/`

**Quick Start:**
```bash
cd gscam_stress
make benchmark-quick     # 6 configs, ~8 minutes
make benchmark-analyze   # View results
```

**See [gscam_stress/README.md](gscam_stress/README.md) for complete workflow and detailed usage.**

### 2. Autoware Planning Simulator

Full-stack Autoware testing with systemd-based service management.

**Features:**
- Launch Autoware with CycloneDDS or Zenoh
- Systemd service management (start, stop, status, logs)
- Environment wrapper scripts for manual ROS commands
- No orphan processes

**Quick Start:**
```bash
# CycloneDDS
make start-sim-cyclonedds
make status-sim-cyclonedds
make stop-sim-cyclonedds

# Zenoh (requires router)
make start-zenoh-router
make start-sim-zenoh
make stop-all
```

**Note:** Planning simulator documentation is available in the Makefile. Run `make help` for available commands.

## References

- [ros2 systemd documentation](https://github.com/ubuntu-robotics/ros2_systemd)
- [gscam_stress Benchmark Suite](gscam_stress/README.md)
- [Latest Benchmark Results](gscam_stress/results/run_2025-10-22_14-40-25/summary.csv)

---
*Updated: 2025-10-22*
