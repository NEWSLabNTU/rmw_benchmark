#!/usr/bin/env python3
"""
RMW Benchmark Results Analyzer

Analyzes CSV results from boundary testing and generates:
- Comparison tables
- Frame loss boundary identification
- Performance metrics comparison
- Optional visualization (if matplotlib available)
"""

import csv
import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple
import argparse


class Colors:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'


def load_summary_csv(csv_path: str) -> List[Dict]:
    """Load summary CSV file and return list of test results"""
    results = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric fields
            try:
                row['fps'] = int(row['fps'])
                row['data_rate_mbps'] = float(row['data_rate_mbps'])
                row['avg_frame_loss_pct'] = float(row['avg_frame_loss_pct'])
                row['max_frame_loss_pct'] = float(row['max_frame_loss_pct'])
                row['avg_latency_ms'] = float(row['avg_latency_ms'])
                row['max_latency_ms'] = float(row['max_latency_ms'])
                row['avg_message_rate'] = float(row['avg_message_rate'])
                row['test_duration_sec'] = int(row['test_duration_sec'])
                results.append(row)
            except (ValueError, KeyError) as e:
                print(f"{Colors.YELLOW}Warning: Skipping invalid row: {e}{Colors.END}")
                continue
    return results


def find_boundaries(results: List[Dict], rmw_type: str) -> Dict[str, Tuple]:
    """Find frame drop boundaries for given RMW implementation"""
    rmw_results = [r for r in results if r['rmw_type'] == rmw_type]
    rmw_results.sort(key=lambda x: x['data_rate_mbps'])

    boundaries = {
        'excellent': None,    # < 0.1%
        'good': None,         # 0.1% - 1%
        'moderate': None,     # 1% - 5%
        'severe': None,       # > 5%
    }

    last_excellent = None
    first_good = None
    first_moderate = None
    first_severe = None

    for result in rmw_results:
        loss = result['avg_frame_loss_pct']

        if loss < 0.1:
            last_excellent = result
        elif 0.1 <= loss < 1.0 and first_good is None:
            first_good = result
        elif 1.0 <= loss < 5.0 and first_moderate is None:
            first_moderate = result
        elif loss >= 5.0 and first_severe is None:
            first_severe = result

    boundaries['excellent'] = last_excellent
    boundaries['good'] = first_good
    boundaries['moderate'] = first_moderate
    boundaries['severe'] = first_severe

    return boundaries


def print_summary_table(results: List[Dict]):
    """Print formatted summary table"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*100}{Colors.END}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'Test Results Summary':^100}{Colors.END}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*100}{Colors.END}\n")

    # Group by RMW type
    cyclone_results = [r for r in results if r['rmw_type'] == 'cyclonedds']
    zenoh_results = [r for r in results if r['rmw_type'] == 'zenoh']

    for rmw_type, rmw_results in [('CycloneDDS', cyclone_results), ('Zenoh', zenoh_results)]:
        if not rmw_results:
            continue

        print(f"{Colors.CYAN}{Colors.BOLD}=== {rmw_type} ==={Colors.END}")
        print(f"{'Config':<30} {'Resolution':<15} {'FPS':>4} {'Rate(MB/s)':>11} {'Loss%':>8} {'Lat(ms)':>9}")
        print("-" * 100)

        for result in sorted(rmw_results, key=lambda x: x['data_rate_mbps']):
            loss = result['avg_frame_loss_pct']
            config = result['config'][:28]
            resolution = result['resolution']
            fps = result['fps']
            rate = result['data_rate_mbps']
            latency = result['avg_latency_ms']

            # Color code based on frame loss
            if loss < 0.1:
                color = Colors.GREEN
            elif loss < 1.0:
                color = Colors.YELLOW
            elif loss < 5.0:
                color = Colors.RED
            else:
                color = Colors.RED + Colors.BOLD

            print(f"{config:<30} {resolution:<15} {fps:>4} {rate:>10.1f} {color}{loss:>7.2f}%{Colors.END} {latency:>8.2f}")

        print()


def print_boundary_analysis(results: List[Dict]):
    """Print frame drop boundary analysis"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*100}{Colors.END}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'Frame Drop Boundary Analysis':^100}{Colors.END}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*100}{Colors.END}\n")

    for rmw_type, rmw_name in [('cyclonedds', 'CycloneDDS'), ('zenoh', 'Zenoh')]:
        print(f"{Colors.CYAN}{Colors.BOLD}=== {rmw_name} ==={Colors.END}\n")

        boundaries = find_boundaries(results, rmw_type)

        if boundaries['excellent']:
            r = boundaries['excellent']
            print(f"{Colors.GREEN}✓ Excellent (< 0.1% loss):{Colors.END}")
            print(f"  Max working: {r['resolution']}@{r['fps']}fps (~{r['data_rate_mbps']:.0f} MB/s)")
            print(f"  Frame loss: {r['avg_frame_loss_pct']:.3f}%, Latency: {r['avg_latency_ms']:.2f}ms\n")

        if boundaries['good']:
            r = boundaries['good']
            print(f"{Colors.YELLOW}⚠ Good (0.1-1% loss):{Colors.END}")
            print(f"  First degradation: {r['resolution']}@{r['fps']}fps (~{r['data_rate_mbps']:.0f} MB/s)")
            print(f"  Frame loss: {r['avg_frame_loss_pct']:.3f}%, Latency: {r['avg_latency_ms']:.2f}ms\n")

        if boundaries['moderate']:
            r = boundaries['moderate']
            print(f"{Colors.RED}⚠ Moderate (1-5% loss):{Colors.END}")
            print(f"  First moderate: {r['resolution']}@{r['fps']}fps (~{r['data_rate_mbps']:.0f} MB/s)")
            print(f"  Frame loss: {r['avg_frame_loss_pct']:.3f}%, Latency: {r['avg_latency_ms']:.2f}ms\n")

        if boundaries['severe']:
            r = boundaries['severe']
            print(f"{Colors.RED}{Colors.BOLD}✗ Severe (> 5% loss):{Colors.END}")
            print(f"  Unusable: {r['resolution']}@{r['fps']}fps (~{r['data_rate_mbps']:.0f} MB/s)")
            print(f"  Frame loss: {r['avg_frame_loss_pct']:.3f}%, Latency: {r['avg_latency_ms']:.2f}ms\n")

        print()


def print_comparison(results: List[Dict]):
    """Print side-by-side comparison of CycloneDDS vs Zenoh"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*100}{Colors.END}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'CycloneDDS vs Zenoh Comparison':^100}{Colors.END}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*100}{Colors.END}\n")

    # Group results by config
    config_map = {}
    for result in results:
        config = result['config']
        if config not in config_map:
            config_map[config] = {}
        config_map[config][result['rmw_type']] = result

    # Print comparison
    print(f"{'Config':<30} {'Resolution':<15} {'CycloneDDS Loss':>16} {'Zenoh Loss':>13} {'Winner':<10}")
    print("-" * 100)

    for config in sorted(config_map.keys(), key=lambda c: config_map[c].get('cyclonedds', config_map[c].get('zenoh', {})).get('data_rate_mbps', 0)):
        if 'cyclonedds' in config_map[config] and 'zenoh' in config_map[config]:
            cyclone = config_map[config]['cyclonedds']
            zenoh = config_map[config]['zenoh']

            resolution = cyclone['resolution']
            cyclone_loss = cyclone['avg_frame_loss_pct']
            zenoh_loss = zenoh['avg_frame_loss_pct']

            # Determine winner
            if abs(cyclone_loss - zenoh_loss) < 0.01:
                winner = "Tie"
                winner_color = Colors.BLUE
            elif cyclone_loss < zenoh_loss:
                winner = "CycloneDDS"
                winner_color = Colors.GREEN
            else:
                winner = "Zenoh"
                winner_color = Colors.GREEN

            print(f"{config[:28]:<30} {resolution:<15} {cyclone_loss:>14.2f}% {zenoh_loss:>12.2f}% {winner_color}{winner:<10}{Colors.END}")

    print()


def generate_findings_report(results: List[Dict], output_path: str):
    """Generate detailed findings report"""
    with open(output_path, 'w') as f:
        f.write("="*80 + "\n")
        f.write("RMW Benchmark Findings Report\n")
        f.write("="*80 + "\n\n")

        for rmw_type, rmw_name in [('cyclonedds', 'CycloneDDS'), ('zenoh', 'Zenoh')]:
            f.write(f"\n{rmw_name} Frame Drop Boundaries\n")
            f.write("-" * 80 + "\n\n")

            boundaries = find_boundaries(results, rmw_type)

            if boundaries['excellent']:
                r = boundaries['excellent']
                f.write(f"Maximum Configuration (< 0.1% loss):\n")
                f.write(f"  Resolution: {r['resolution']}@{r['fps']}fps\n")
                f.write(f"  Data Rate: ~{r['data_rate_mbps']:.0f} MB/s\n")
                f.write(f"  Frame Loss: {r['avg_frame_loss_pct']:.3f}%\n")
                f.write(f"  Latency: {r['avg_latency_ms']:.2f}ms (avg), {r['max_latency_ms']:.2f}ms (max)\n\n")

            if boundaries['good']:
                r = boundaries['good']
                f.write(f"First Degradation Point (0.1-1% loss):\n")
                f.write(f"  Resolution: {r['resolution']}@{r['fps']}fps\n")
                f.write(f"  Data Rate: ~{r['data_rate_mbps']:.0f} MB/s\n")
                f.write(f"  Frame Loss: {r['avg_frame_loss_pct']:.3f}%\n")
                f.write(f"  Latency: {r['avg_latency_ms']:.2f}ms (avg), {r['max_latency_ms']:.2f}ms (max)\n\n")

            if boundaries['moderate']:
                r = boundaries['moderate']
                f.write(f"Moderate Degradation (1-5% loss):\n")
                f.write(f"  Resolution: {r['resolution']}@{r['fps']}fps\n")
                f.write(f"  Data Rate: ~{r['data_rate_mbps']:.0f} MB/s\n")
                f.write(f"  Frame Loss: {r['avg_frame_loss_pct']:.3f}%\n")
                f.write(f"  Latency: {r['avg_latency_ms']:.2f}ms (avg), {r['max_latency_ms']:.2f}ms (max)\n\n")

    print(f"{Colors.GREEN}Findings report saved: {output_path}{Colors.END}")


def main():
    parser = argparse.ArgumentParser(description='Analyze RMW benchmark results')
    parser.add_argument('summary_csv', help='Path to summary.csv file')
    parser.add_argument('--report', '-r', help='Output path for findings report')
    args = parser.parse_args()

    if not os.path.exists(args.summary_csv):
        print(f"{Colors.RED}Error: File not found: {args.summary_csv}{Colors.END}")
        sys.exit(1)

    # Load results
    print(f"{Colors.BLUE}Loading results from: {args.summary_csv}{Colors.END}")
    results = load_summary_csv(args.summary_csv)

    if not results:
        print(f"{Colors.RED}Error: No valid results found in CSV{Colors.END}")
        sys.exit(1)

    print(f"{Colors.GREEN}Loaded {len(results)} test results{Colors.END}")

    # Print analyses
    print_summary_table(results)
    print_boundary_analysis(results)
    print_comparison(results)

    # Generate report if requested
    if args.report:
        generate_findings_report(results, args.report)

    print(f"\n{Colors.GREEN}{Colors.BOLD}Analysis complete!{Colors.END}\n")


if __name__ == '__main__':
    main()
