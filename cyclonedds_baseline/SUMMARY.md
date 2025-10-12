# CycloneDDS Baseline Summary

## Execution Details
- **RMW Implementation**: rmw_cyclonedds_cpp
- **ROS Domain ID**: 188
- **Launch Command**: `make start-sim-cyclonedds`
- **Initialization Time**: 90 seconds
- **Date**: 2025-10-11

## System State Captured

### Nodes
- **Total Nodes**: 169
- **File**: `cyclonedds_baseline/nodes.txt`
- **Note**: Warning present about duplicate node names

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
- **File**: `cyclonedds_baseline/topics.txt`

Key topic namespaces:
- `/api/*` - API topics
- `/control/*` - Control topics
- `/planning/*` - Planning topics
- `/perception/*` - Perception topics
- `/vehicle/*` - Vehicle status topics
- `/tf` and `/tf_static` - Transform topics

### Services
- **Total Services**: 710
- **File**: `cyclonedds_baseline/services.txt`

Includes parameter services for all nodes and functional services like:
- `/localization/initialize`
- `/map/get_differential_pointcloud_map`
- `/map/get_partial_pointcloud_map`

## Launch Status

The simulator launched successfully:
- All major Autoware components initialized
- Transform listeners operational
- Planning, control, and perception pipelines active
- Duplicate node warnings present (likely from transform listeners)

## Files Generated
- `nodes.txt` - Complete list of active nodes
- `topics.txt` - Complete list of topics with message types
- `services.txt` - Complete list of services with types
- `launch.log` - Full launch output

---
*Updated: 2025-10-11*
