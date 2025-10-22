# RMW Zenoh vs CycloneDDS Benchmark

This repository contains comprehensive benchmarking tools and results comparing RMW implementations (CycloneDDS vs Zenoh) for high-bandwidth sensor streaming in ROS 2.

## Repository Overview

- **Location**: `gscam_stress/`
- **Purpose**: Compare CycloneDDS vs Zenoh for high-bandwidth sensor streaming
- **Test Coverage**: 14 configurations from 13 MB/s to 1424 MB/s (4K@60fps)

## Key Findings

**Latest benchmark run**: `gscam_stress/results/run_2025-10-22_14-40-25/` (with Zenoh 1.4 GB pool)

Based on comprehensive testing across 14 configurations (13 MB/s to 1424 MB/s) with **shared memory enabled** for both implementations:

1. **CycloneDDS performance (with Iceoryx shared memory - 1.4 GB)**
   - **Perfect 0.00% frame loss** across nearly all data rates (13-1068 MB/s)
   - Minor degradation only at extreme 4K@60fps: 0.06% loss at 1424 MB/s
   - Low, stable latency: 0.63ms @ 26 MB/s, 13.19ms @ 1424 MB/s
   - Configuration: 1.4 GB Iceoryx memory pools (5 tiered pools: 1KB-25MB)

2. **Zenoh performance (with 1.4 GB shared memory pool)**
   - **Perfect 0.00-0.04% frame loss up to 1068 MB/s (4K@45fps)**
   - **Severe degradation at 4K@60fps: 36.75% loss at 1424 MB/s**
   - Higher latency than CycloneDDS at 4K loads: 31.75ms @ 1068 MB/s
   - Configuration: 1.4 GB unified shared memory pool (matches CycloneDDS total)
   - **Pool Size Paradox**: Larger pools perform WORSE at extreme loads
     - 256 MB: 6.09% loss @ 4K@45fps, 17.69% loss @ 4K@60fps
     - 512 MB: 1.64% loss @ 4K@45fps, 22.62% loss @ 4K@60fps
     - 1.4 GB: 0.04% loss @ 4K@45fps, **36.75% loss @ 4K@60fps** (WORST!)
   - **Key insight**: Unified pool has memory management overhead; tiered pools are superior

3. **Recommendations**
   - **For HD/Full HD/2K streaming (up to 633 MB/s)**: Either implementation works perfectly
   - **For 4K@30fps (712 MB/s)**: Either implementation works excellently
   - **For 4K@45fps (1068 MB/s)**: Either implementation works excellently (both <0.1% loss)
   - **For 4K@60fps (1424 MB/s)**: **Use CycloneDDS** - Zenoh suffers from pool size paradox (36.75% loss)
   - **For simplicity (no daemon required)**: Zenoh is excellent up to 4K@45fps
   - **Architecture matters**: CycloneDDS's tiered pools outperform Zenoh's unified pool at extreme loads
   - Always test both implementations with your specific hardware and use case
   - See `gscam_stress/results/run_2025-10-22_14-40-25/summary.csv` for raw data

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
cat results/run_2025-10-22_14-40-25/summary.csv
```

## Using Benchmark Results

The benchmark data is stored in `gscam_stress/results/run_2025-10-22_14-40-25/`:
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

- **CycloneDDS**: 1.4 GB Iceoryx shared memory (5 tiered pools: 1KB-25MB) - see `gscam_stress/cyclonedds/roudi_config.toml`
- **Zenoh**: 1.4 GB unified pool (matches CycloneDDS total) - see `gscam_stress/rmw_config/zenoh_shm.json5`
- Both implementations use shared memory for zero-copy transport
- **Key difference**: Tiered pools (CycloneDDS) vs unified pool (Zenoh) - architecture matters more than size

## Git Configuration

- Main branch: `main`
- Repository contains configuration files and scripts for automated benchmarking

## Notes

- Benchmark results are hardware and configuration dependent
- For latest results, see `gscam_stress/results/run_2025-10-22_14-40-25/summary.csv`
- Run your own benchmarks to validate performance for your specific use case
- **Pool Size Paradox discovered**: Zenoh's larger pools perform WORSE at extreme loads
  - 256 MB → 512 MB → 1.4 GB improved 4K@45fps (6.09% → 1.64% → 0.04%)
  - But worsened 4K@60fps (17.69% → 22.62% → **36.75%**)
  - CycloneDDS's tiered pool architecture is fundamentally more efficient
- 4K@60fps requires CycloneDDS - Zenoh shows 36.75% loss with 1.4 GB pool (pool size paradox)

---
*Last updated: 2025-10-22*
