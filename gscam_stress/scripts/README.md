# Benchmark Scripts

This directory contains automated testing scripts for RMW (CycloneDDS vs Zenoh) benchmarking.

## Scripts Overview

### benchmark_rmw.sh
Single test automation script - runs one configuration and reports results.

**Usage:**
```bash
./benchmark_rmw.sh <rmw_type> <config> <duration_sec>
```

**Examples:**
```bash
# Run CycloneDDS with high.conf for 60 seconds
./benchmark_rmw.sh cyclonedds high.conf 60

# Run Zenoh with 2560x1440_60fps.conf for 120 seconds
./benchmark_rmw.sh zenoh 2560x1440_60fps.conf 120

# Works from any directory
/path/to/gscam_stress/scripts/benchmark_rmw.sh zenoh medium.conf 30
```

### run_boundary_test.sh
Full test suite runner - tests all 14 configurations with both RMW implementations (28 tests total).

**Usage:**
```bash
./run_boundary_test.sh [duration_per_test]
```

**Examples:**
```bash
# Run full suite with 60s per test (~35 minutes total)
./run_boundary_test.sh 60

# Quick validation with 30s per test (~18 minutes)
./run_boundary_test.sh 30
```

**Output:**
- Results saved in `../results/run_TIMESTAMP/`
- Summary CSV with all test results
- Automatic boundary analysis

### quick_boundary_test.sh
Quick validation script - tests 6 key configurations (12 tests total).

**Usage:**
```bash
./quick_boundary_test.sh
```

**Duration:** ~8 minutes (30s per test)

**Configs tested:**
- low.conf (640×480@15fps)
- medium.conf (1280×720@30fps)
- high.conf (1920×1080@30fps)
- 1080p60.conf (1920×1080@60fps)
- 2560x1440_30fps.conf
- 2560x1440_60fps.conf

### analyze_results.py
Python analysis tool - parses summary CSV and generates detailed reports.

**Usage:**
```bash
./analyze_results.py <summary_csv> [--report output.txt]
```

**Examples:**
```bash
# Analyze results and display in terminal
./analyze_results.py ../results/run_2025-10-12_19-18-53/summary.csv

# Generate findings report
./analyze_results.py ../results/run_2025-10-12_19-18-53/summary.csv --report findings.txt
```

**Features:**
- Color-coded performance tables
- Boundary analysis (excellent/good/moderate/severe)
- CycloneDDS vs Zenoh comparison
- Optional text report generation

## Typical Workflow

### Quick Test (8 minutes)
```bash
cd /path/to/gscam_stress
scripts/quick_boundary_test.sh
scripts/analyze_results.py results/quick_run_*/summary.csv
```

### Full Benchmark (35 minutes)
```bash
cd /path/to/gscam_stress
scripts/run_boundary_test.sh 60
scripts/analyze_results.py results/run_*/summary.csv --report findings.txt
```

### Single Configuration Test
```bash
cd /path/to/gscam_stress
scripts/benchmark_rmw.sh cyclonedds extreme.conf 60
scripts/benchmark_rmw.sh zenoh extreme.conf 60
```

## Path Independence

All scripts use the `SCRIPT_DIR` trick and work regardless of your current working directory:

```bash
# All of these work:
cd /path/to/gscam_stress && scripts/benchmark_rmw.sh cyclonedds high.conf 60
cd /tmp && /path/to/gscam_stress/scripts/benchmark_rmw.sh cyclonedds high.conf 60
/path/to/gscam_stress/scripts/benchmark_rmw.sh cyclonedds high.conf 60
```

Scripts automatically:
- Locate the project root directory
- Find configuration files
- Create results in correct locations
- Call other scripts with proper paths

## Output Structure

```
gscam_stress/
├── scripts/              # Benchmark automation scripts (this directory)
├── cyclonedds/          # CycloneDDS test CSVs (created by tests)
├── zenoh/               # Zenoh test CSVs (created by tests)
└── results/             # Organized test results
    ├── run_TIMESTAMP/
    │   ├── cyclonedds/
    │   │   └── config_timestamp.csv
    │   ├── zenoh/
    │   │   └── config_timestamp.csv
    │   └── summary.csv
    └── quick_run_TIMESTAMP/
        ├── cyclonedds/
        ├── zenoh/
        └── summary.csv
```

## Requirements

- ROS 2 Humble
- gscam package built
- CycloneDDS and Zenoh RMW implementations
- bc (for math calculations)
- Python 3 (for analysis tool)

## See Also

- [BENCHMARK_FINDINGS.md](../BENCHMARK_FINDINGS.md) - Detailed benchmark results
- [STRESS_TEST_CONFIG.md](../STRESS_TEST_CONFIG.md) - Configuration guide
- [README.md](../README.md) - Main project README
