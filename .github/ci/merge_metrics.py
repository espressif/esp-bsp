# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import glob
import json
import os
import sys


def main():
    benchmark_dir_path = sys.argv[1]
    if not os.path.exists(benchmark_dir_path):
        print(f'Specified benchmark directory "{os.path.basename(benchmark_dir_path)}" does not exist')
        sys.exit(1)
    build_dirs = glob.glob(f'{benchmark_dir_path}/build*/')
    for build_path in build_dirs:
        print(f'Processing build directory: {build_path}')

        benchmark_files = glob.glob(os.path.join(build_path, 'benchmark*.json'))
        if not benchmark_files:
            print(f'WARNING: No benchmark*.json found in {build_path}, skipping...')
            continue

        benchmark_result_path = benchmark_files[0]
        print(f'Loading benchmark results from: {benchmark_result_path}')

        with open(benchmark_result_path) as f:
            benchmark_result = json.load(f)

        size_json_path = os.path.join(build_path, 'size.json')
        print(f'Loading binary size data from: {size_json_path}')
        with open(size_json_path) as f:
            benchmark_result['binary_size'] = json.load(f)

        directory, filename = os.path.split(benchmark_result_path)
        metrics_file_path = os.path.join(directory, filename.replace('benchmark', 'metrics'))

        with open(metrics_file_path, 'w') as f:
            json.dump(benchmark_result, f, indent=4)
        print(f'Metrics written to: {metrics_file_path}')


if __name__ == "__main__":
    main()
