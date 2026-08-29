# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
"""Aggregate BSP benchmark + size data into one InfluxDB-friendly file.

Output: <project_dir>/bsp_metrics.json with two top-level measurements
(`lvgl_benchmark`, `bsp_binary_size`), each holding a flat `data` array
where every row carries its own tags (board, idf_version, lvgl_version,
plus scene or region) and numeric fields.
"""

import json
import os
import sys
from pathlib import Path

OUTPUT_NAME = 'bsp_metrics.json'


def _load_json(path: Path) -> dict | None:
    try:
        with path.open() as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f'WARNING: cannot read {path}: {e}')
        return None


def _benchmark_rows(benchmark: dict, idf_version: str) -> list[dict]:
    base = {
        'board': benchmark.get('board', 'unknown'),
        'idf_version': idf_version,
        'lvgl_version': benchmark.get('lvgl_version', ''),
    }
    return [{**base, **test} for test in benchmark.get('tests', [])]


def _size_rows(size_json: dict, board: str, idf_version: str, lvgl_version: str) -> list[dict]:
    return [
        {
            'board': board,
            'idf_version': idf_version,
            'lvgl_version': lvgl_version,
            'region': r['name'],
            'total': r['total'],
            'used': r['used'],
            'free': r['free'],
        }
        for r in size_json.get('layout', [])
    ]


def _process_build_dir(build_dir: Path, idf_version: str) -> tuple[list[dict], list[dict]]:
    benchmark_files = sorted(build_dir.glob('benchmark_*.json'))
    size_file = build_dir / 'size.json'
    if not benchmark_files or not size_file.is_file():
        print(f'WARNING: incomplete build dir, skipping: {build_dir}')
        return [], []

    benchmark = _load_json(benchmark_files[0])
    size_json = _load_json(size_file)
    if benchmark is None or size_json is None:
        print(f'WARNING: unparseable JSON, skipping: {build_dir}')
        return [], []

    return (
        _benchmark_rows(benchmark, idf_version),
        _size_rows(
            size_json,
            benchmark.get('board', 'unknown'),
            idf_version,
            benchmark.get('lvgl_version', ''),
        ),
    )


def run(project_dir: Path) -> None:
    if not project_dir.is_dir():
        print(f'ERROR: project dir does not exist: {project_dir}')
        sys.exit(1)

    idf_version = os.environ.get('IDF_VERSION', '')
    if not idf_version:
        print('WARNING: IDF_VERSION not set; tag will be empty')

    build_dirs = sorted(p for p in project_dir.glob('build_*') if p.is_dir())
    if not build_dirs:
        print(f'ERROR: no build_* directories under {project_dir}')
        sys.exit(2)

    bench_all: list[dict] = []
    size_all: list[dict] = []
    for build_dir in build_dirs:
        b, s = _process_build_dir(build_dir, idf_version)
        bench_all.extend(b)
        size_all.extend(s)

    metrics = {
        'lvgl_benchmark': {'data': bench_all},
        'bsp_binary_size': {'data': size_all},
    }
    output = project_dir / OUTPUT_NAME
    output.write_text(json.dumps(metrics, indent=4))
    print(f'Wrote {len(bench_all)} benchmark rows + {len(size_all)} size rows to {output}')

    if not bench_all and not size_all:
        sys.exit(3)


def main() -> None:
    if len(sys.argv) != 2:
        print('Usage: merge_metrics.py <project_dir>')
        sys.exit(1)
    run(Path(sys.argv[1]))


if __name__ == '__main__':
    main()
