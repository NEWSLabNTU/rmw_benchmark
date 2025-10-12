# Autoware RMW Zenoh Benchmark

Comprehensive benchmarking tools for comparing ROS 2 RMW implementations (CycloneDDS vs Zenoh) with high-bandwidth sensor streaming tests.

## Overview

This repository provides:
- **gscam_stress** - Automated benchmark suite for RMW comparison (recommended)
  - 14 test configurations from 13 MB/s to 1424 MB/s (4K@60fps)
  - Comprehensive performance analysis and recommendations
  - See [gscam_stress/README.md](gscam_stress/README.md) for details
- **Autoware planning simulator** - Full-stack testing with Autoware
  - Makefile to launch with **CycloneDDS** or **Zenoh**
  - Systemd service management (start, stop, status, logs)
  - Environment wrapper scripts for manual ROS commands

## Quick Start - Benchmark Suite (Recommended)

The automated benchmark suite provides comprehensive RMW comparison:

```bash
cd gscam_stress

# Quick test (6 configs, ~8 minutes)
make benchmark-quick

# Full suite (14 configs, ~35 minutes)
make benchmark

# Analyze results
make benchmark-analyze

# View results
cat results/run_2025-10-13_01-02-46/summary.csv
```

**Key Findings**: Based on testing across 14 configurations (results in `gscam_stress/results/run_2025-10-13_01-02-46/`), CycloneDDS shows consistent 0.00-0.97% frame loss, while Zenoh achieves 0.00% frame loss up to 178 MB/s. See detailed metrics in the results directory.

## Quick Start - Planning Simulator

For full-stack Autoware testing:

## Benefits of systemd-based Launching

Using `ros2 systemd launch/run` instead of direct `ros2 launch` provides:
- ✓ **No orphan processes** - systemd tracks all child processes
- ✓ **Clean shutdown** - proper cleanup when stopping services
- ✓ **Service management** - start, stop, restart, status commands
- ✓ **Logging** - centralized log management via systemd
- ✓ **Automatic restart** - can configure services to restart on failure

## Planning Simulator Usage

```bash
# CycloneDDS workflow
make start-sim-cyclonedds
make status-sim-cyclonedds
make logs-sim-cyclonedds
make stop-sim-cyclonedds

# Zenoh workflow (router required first)
make start-zenoh-router
make start-sim-zenoh
make status-all
make stop-all

# Show all available commands
make help
```

---

# Planning Simulator Details

The planning simulator provides full-stack Autoware testing with different RMW implementations using systemd services.

## Makefile Targets

### CycloneDDS

```bash
make start-sim-cyclonedds     # Start Autoware with CycloneDDS
make stop-sim-cyclonedds      # Stop CycloneDDS simulation
make status-sim-cyclonedds    # Show service status
make logs-sim-cyclonedds      # View logs
```

Service details:
- Service name: `autoware-planning-cyclonedds`
- RMW: `rmw_cyclonedds_cpp`
- ROS Domain ID: 188

### Zenoh

```bash
# Step 1: Start Zenoh router (REQUIRED)
make start-zenoh-router       # Start Zenoh router daemon

# Step 2: Start planning simulator
make start-sim-zenoh          # Start Autoware with Zenoh

# Management
make stop-sim-zenoh           # Stop Zenoh simulation
make stop-zenoh-router        # Stop Zenoh router
make status-sim-zenoh         # Show simulation status
make status-zenoh-router      # Show router status
make logs-sim-zenoh           # View simulation logs
make logs-zenoh-router        # View router logs
```

Service details:
- Router service: `zenoh-router` (listens on port 7447)
- Planning service: `autoware-planning-zenoh`
- RMW: `rmw_zenoh_cpp`
- ROS Domain ID: 189

**Important:** The Zenoh router MUST be running before starting the planning simulator. The Makefile will check and error if not.

### Management Targets

```bash
make stop-all                 # Stop all services
make status-all               # Show all service statuses
make help                     # Show available targets (default)
```

## Environment Wrapper Scripts

For manual ROS commands, use the environment wrapper scripts:

```bash
# Use Zenoh RMW
./zenoh_env.sh ros2 node list
./zenoh_env.sh ros2 topic hz /clock

# Use CycloneDDS RMW
./cyclonedds_env.sh ros2 node list
./cyclonedds_env.sh ros2 topic list
```

These wrapper scripts set up the correct environment (RMW, domain ID, workspace) for running ROS commands manually. They work from any directory.

## Additional Service Management Commands

All services are also accessible via `ros2 systemd` commands:

### View Service Status
```bash
ros2 systemd status autoware-planning-cyclonedds
ros2 systemd status autoware-planning-zenoh
ros2 systemd status zenoh-router
```

### View Service Logs
```bash
# View logs
ros2 systemd logs autoware-planning-cyclonedds

# View last 100 lines
ros2 systemd logs autoware-planning-cyclonedds --lines 100

# Follow logs in real-time
journalctl --user -u ros2-autoware-planning-cyclonedds.service -f
```

### Stop a Service
```bash
ros2 systemd stop autoware-planning-cyclonedds
ros2 systemd stop autoware-planning-zenoh
ros2 systemd stop zenoh-router
```

### List All ROS2 Systemd Services
```bash
ros2 systemd list
```

### Remove a Service
```bash
ros2 systemd remove autoware-planning-cyclonedds
```

## Typical Workflows

### Running with CycloneDDS

```bash
# 1. Launch
make start-sim-cyclonedds

# 2. Check status
make status-sim-cyclonedds

# 3. Monitor logs (in another terminal)
make logs-sim-cyclonedds

# 4. Run ROS commands (in another terminal)
./cyclonedds_env.sh ros2 node list
./cyclonedds_env.sh ros2 topic hz /clock

# 5. Stop when done
make stop-sim-cyclonedds
```

### Running with Zenoh

```bash
# 1. Start Zenoh router
make start-zenoh-router

# 2. Launch Autoware
make start-sim-zenoh

# 3. Check status
make status-all

# 4. Monitor logs (in another terminal)
make logs-zenoh-router
make logs-sim-zenoh

# 5. Run ROS commands (in another terminal)
./zenoh_env.sh ros2 node list
./zenoh_env.sh ros2 topic hz /clock

# 6. Stop when done
make stop-all
```

### Benchmarking Both RMWs

```bash
# Test CycloneDDS
make start-sim-cyclonedds
sleep 60  # Wait for initialization
./cyclonedds_env.sh ros2 node list > cyclonedds_nodes.txt
./cyclonedds_env.sh ros2 topic list > cyclonedds_topics.txt
make stop-sim-cyclonedds

# Test Zenoh
make start-zenoh-router
make start-sim-zenoh
sleep 120  # Wait longer for Zenoh
./zenoh_env.sh ros2 node list > zenoh_nodes.txt
./zenoh_env.sh ros2 topic list > zenoh_topics.txt
make stop-all

# Compare
diff cyclonedds_nodes.txt zenoh_nodes.txt
```

## Configuration

Launch target settings:
- **ROS Domain ID (CycloneDDS)**: 188
- **ROS Domain ID (Zenoh)**: 189 (different domain to avoid interference)
- **Display**: :1 (for RViz)
- **Map**: `~/autoware_map/sample-map-planning`
- **Vehicle**: `sample_vehicle`
- **Sensor**: `sample_sensor_kit`

To modify these, edit the variables at the top of the Makefile.

## Troubleshooting

### Service won't start
```bash
# Check detailed status
systemctl --user status ros2-autoware-planning-zenoh.service

# View full logs
journalctl --user -u ros2-autoware-planning-zenoh.service --no-pager
```

### Port 7447 already in use (Zenoh)
```bash
# Check what's using the port
ss -tulpn | grep 7447

# Stop Docker Zenoh bridge if running
docker stop zenoh-bridge-autoware_zenoh_bridge-1

# Or stop the router service
make stop-zenoh-router
```

### Zenoh simulation won't start
The Makefile checks if the router is running. If you see an error:
```
ERROR: Zenoh router is not running. Start it first with: make start-zenoh-router
```

Simply start the router first:
```bash
make start-zenoh-router
make start-sim-zenoh
```

### Orphan processes still exist
This shouldn't happen with systemd-based launching! If it does:
```bash
# Check systemd service status
ros2 systemd list

# Stop all services
make stop-all

# Verify no ROS processes remain
ps aux | grep ros2
```

## Files in This Directory

- `Makefile` - Main interface for launching and managing planning simulator services
- `cyclonedds_env.sh` - Environment wrapper for CycloneDDS commands
- `zenoh_env.sh` - Environment wrapper for Zenoh commands
- `gscam_stress/` - **Automated benchmark suite (recommended)**
- `README.md` - This file

## Benchmark Suite

For comprehensive RMW performance testing, see the **[gscam_stress](gscam_stress/)** directory:
- Automated benchmark suite with 14 test configurations
- Performance analysis from 13 MB/s to 1424 MB/s
- Latest results in `gscam_stress/results/run_2025-10-13_01-02-46/`
- CSV output and analysis tools

See [gscam_stress/README.md](gscam_stress/README.md) for details.

## References

- [ros2 systemd documentation](https://github.com/ubuntu-robotics/ros2_systemd)
- [gscam_stress Benchmark Suite](gscam_stress/README.md)
- [Latest Benchmark Results](gscam_stress/results/run_2025-10-13_01-02-46/summary.csv)

---
*Updated: 2025-10-13*
