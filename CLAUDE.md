# RMW Zenoh vs CycloneDDS Benchmark

This repository contains comprehensive benchmarking tools and results comparing RMW implementations (CycloneDDS vs Zenoh) for high-bandwidth sensor streaming in ROS 2.

## Repository Overview

- **Location**: `gscam_stress/`
- **Purpose**: Compare CycloneDDS vs Zenoh for high-bandwidth sensor streaming
- **Test Coverage**: 14 configurations from 13 MB/s to 1424 MB/s (4K@60fps)

## Key Findings

**Latest benchmark run**: `gscam_stress/results/run_2025-10-13_01-02-46/`

Based on comprehensive testing across 14 configurations (13 MB/s to 1424 MB/s):

1. **CycloneDDS performance**
   - Consistent 0.00-0.97% frame loss across all data rates
   - Predictable performance characteristics
   - Tested from 640x480@15fps up to 4K@60fps (1424 MB/s)

2. **Zenoh performance**
   - 0.00% frame loss from 13 MB/s to 267 MB/s (Full HD@45fps)
   - Excellent low-latency characteristics in tested range
   - See summary.csv for detailed metrics

3. **Testing recommendation**
   - Always benchmark both implementations for your specific use case
   - Results may vary based on hardware, network topology, and configuration
   - See `gscam_stress/results/run_2025-10-13_01-02-46/summary.csv` for raw data

## Benchmark Usage

```bash
cd gscam_stress

# Quick test (6 configs, ~8 minutes)
make benchmark-quick

# Full suite (14 configs, ~35 minutes)
make benchmark

# Analyze latest results
make benchmark-analyze

# View results
cat results/run_2025-10-13_01-02-46/summary.csv
```

## Using Benchmark Results

The benchmark data is stored in `gscam_stress/results/run_2025-10-13_01-02-46/`:
- `summary.csv` - Aggregated results for all 14 configurations
- `cyclonedds/` - Individual test CSV files for CycloneDDS
- `zenoh/` - Individual test CSV files for Zenoh

Key metrics in the CSV:
- `avg_frame_loss_pct` - Average percentage of frames lost
- `avg_latency_ms` - Average message latency
- `data_rate_mbps` - Actual data throughput

**Recommendation**: Always test both RMW implementations with your specific hardware, network topology, and sensor configurations before deployment.

## Git Configuration

- Main branch: `main`
- Repository contains configuration files and scripts for automated benchmarking

## Notes

- Benchmark results are hardware and configuration dependent
- For latest results, see `gscam_stress/results/run_2025-10-13_01-02-46/summary.csv`
- Run your own benchmarks to validate performance for your specific use case

---
*Last updated: 2025-10-13*
