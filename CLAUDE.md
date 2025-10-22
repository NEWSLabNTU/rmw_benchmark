# RMW Zenoh vs CycloneDDS Benchmark

This repository contains comprehensive benchmarking tools and results comparing RMW implementations (CycloneDDS vs Zenoh) for high-bandwidth sensor streaming in ROS 2.

## Repository Overview

- **Location**: `gscam_stress/`
- **Purpose**: Compare CycloneDDS vs Zenoh for high-bandwidth sensor streaming
- **Test Coverage**: 14 configurations from 13 MB/s to 1424 MB/s (4K@60fps)

## Key Findings

**Latest benchmark run**: `gscam_stress/results/run_2025-10-22_14-00-01/` (with Zenoh 512 MB pool)

Based on comprehensive testing across 14 configurations (13 MB/s to 1424 MB/s) with **shared memory enabled** for both implementations:

1. **CycloneDDS performance (with Iceoryx shared memory - 1.4 GB)**
   - **Perfect 0.00% frame loss** across nearly all data rates (13-1068 MB/s)
   - Minor degradation only at extreme 4K@60fps: 0.17% loss at 1424 MB/s
   - Low, stable latency: 0.61ms @ 26 MB/s, 16.70ms @ 1424 MB/s
   - Configuration: 1.4 GB Iceoryx memory pools (5 tiers: 1KB-25MB)

2. **Zenoh performance (with 512 MB shared memory pool)**
   - **Perfect 0.00-0.05% frame loss up to 712 MB/s (4K@30fps)**
   - Moderate degradation at 4K@45fps: 1.64% loss at 1068 MB/s
   - Severe degradation at 4K@60fps: 22.62% loss at 1424 MB/s
   - Configuration: 512 MB unified shared memory pool (increased from 256 MB)
   - The 256→512 MB increase improved 4K@45fps significantly (6.09%→1.64% loss)

3. **Recommendations**
   - **For HD/Full HD/2K streaming (up to 633 MB/s)**: Either implementation works perfectly
   - **For 4K@30fps (712 MB/s)**: Either implementation works excellently
   - **For 4K@45fps (1068 MB/s)**: Use CycloneDDS for excellent reliability (Zenoh acceptable at 1.64% loss)
   - **For 4K@60fps (1424 MB/s)**: Use CycloneDDS - Zenoh is unusable at this load
   - **For simplicity (no daemon required)**: Zenoh is sufficient up to 4K@30fps
   - Always test both implementations with your specific hardware and use case
   - See `gscam_stress/results/run_2025-10-22_14-00-01/summary.csv` for raw data

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
cat results/run_2025-10-22_14-00-01/summary.csv
```

## Using Benchmark Results

The benchmark data is stored in `gscam_stress/results/run_2025-10-22_14-00-01/`:
- `summary.csv` - Aggregated results for all 14 configurations
- `findings.txt` - Analyzed results with frame drop boundaries
- `cyclonedds/` - Individual test CSV files for CycloneDDS
- `zenoh/` - Individual test CSV files for Zenoh

Key metrics in the CSV:
- `avg_frame_loss_pct` - Average percentage of frames lost
- `avg_latency_ms` - Average message latency
- `data_rate_mbps` - Actual data throughput

**Recommendation**: Always test both RMW implementations with your specific hardware, network topology, and sensor configurations before deployment.

## Configuration Details

- **CycloneDDS**: 1.4 GB Iceoryx shared memory (5 tiered pools) - see `gscam_stress/cyclonedds/roudi_config.toml`
- **Zenoh**: 512 MB unified pool - see `gscam_stress/rmw_config/zenoh_shm.json5`
- Both implementations use shared memory for zero-copy transport

## Git Configuration

- Main branch: `main`
- Repository contains configuration files and scripts for automated benchmarking

## Notes

- Benchmark results are hardware and configuration dependent
- For latest results, see `gscam_stress/results/run_2025-10-22_14-00-01/summary.csv`
- Run your own benchmarks to validate performance for your specific use case
- Zenoh 512 MB pool improved 4K@45fps from 6.09% to 1.64% loss (vs 256 MB)
- 4K@60fps requires CycloneDDS - Zenoh shows 22.62% loss even with 512 MB pool

---
*Last updated: 2025-10-22*
