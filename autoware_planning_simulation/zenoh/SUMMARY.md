# Zenoh Experimental Summary

## Execution Details
- **RMW Implementation**: rmw_zenoh_cpp
- **ROS Domain ID**: 189
- **Launch Command**: `make start-zenoh-router && make start-sim-zenoh`
- **Initialization Time**: 120 seconds (including router startup)
- **Date**: 2025-10-11

## System State Captured

### Nodes
- **Total Nodes**: 135
- **File**: `zenoh_experimental/nodes.txt`
- **Note**: No duplicate node warnings observed

Key node namespaces:
- `/adapi/*` - AD API nodes
- `/control/*` - Control system nodes
- `/planning/*` - Planning system nodes
- `/perception/*` - Perception system nodes
- `/simulation/*` - Simulator nodes
- `/system/*` - System monitoring nodes
- `/map/*` - Map management nodes

### Topics
- **Total Topics**: 516
- **File**: `zenoh_experimental/topics.txt`

Key topic namespaces:
- `/api/*` - API topics
- `/control/*` - Control topics
- `/planning/*` - Planning topics
- `/perception/*` - Perception topics
- `/vehicle/*` - Vehicle status topics
- `/tf` and `/tf_static` - Transform topics

### Services
- **Total Services**: 710
- **File**: `zenoh_experimental/services.txt`

Includes parameter services for all nodes and functional services like:
- `/localization/initialize`
- `/map/get_differential_pointcloud_map`
- `/map/get_partial_pointcloud_map`

## Launch Status

The simulator launched successfully:
- All major Autoware components initialized
- Transform listeners operational
- Planning, control, and perception pipelines active
- Zenoh router daemon running on port 7447

## Comparison with CycloneDDS

- **Nodes**: 135 (Zenoh) vs 169 (CycloneDDS) - **34 fewer nodes**
- **Topics**: 516 (both) - **No difference**
- **Services**: 710 (both) - **No difference**

The reduced node count in Zenoh may indicate different node discovery behavior or lifecycle management compared to CycloneDDS.

## Files Generated
- `nodes.txt` - Complete list of active nodes
- `topics.txt` - Complete list of topics with message types
- `services.txt` - Complete list of services with types
- `router.log` - Zenoh router logs (if captured)
- `launch.log` - Full launch output (if captured)

---
*Updated: 2025-10-11*
