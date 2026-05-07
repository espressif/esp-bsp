# SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import json
from importlib import import_module
from pathlib import Path

import pytest

merge = import_module('merge_metrics')

FIX = Path(__file__).parent / 'fixtures'


def _copytree(src: Path, dst: Path) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    for child in src.iterdir():
        (dst / child.name).write_bytes(child.read_bytes())


def test_aggregates_one_build_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv('IDF_VERSION', 'release-v5.5')
    project = tmp_path / 'proj'
    _copytree(FIX / 'sample_box3', project / 'build_esp_box_3_default')

    merge.run(project)

    out = json.loads((project / 'bsp_metrics.json').read_text())
    assert out['lvgl_benchmark']['data'] == [{
        'board': 'esp_box_3',
        'idf_version': 'release-v5.5',
        'lvgl_version': '9.2.0',
        'scene': 'Widgets demo',
        'avg_cpu': 50,
        'avg_fps': 30,
        'avg_time_ms': 33,
        'render_time_ms': 20,
        'flush_time_ms': 13,
    }]
    assert out['bsp_binary_size']['data'] == [{
        'board': 'esp_box_3',
        'idf_version': 'release-v5.5',
        'lvgl_version': '9.2.0',
        'region': 'Flash Code',
        'total': 2097152,
        'used': 158720,
        'free': 1938432,
    }]


def test_skips_build_dir_missing_size(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setenv('IDF_VERSION', 'release-v5.5')
    project = tmp_path / 'proj'
    (project / 'build_missing_size').mkdir(parents=True)
    src = FIX / 'sample_missing_size' / 'benchmark_x.json'
    (project / 'build_missing_size' / src.name).write_bytes(src.read_bytes())

    with pytest.raises(SystemExit):
        merge.run(project)

    captured = capsys.readouterr().out.lower()
    assert 'skipping' in captured


def test_exits_nonzero_when_no_build_dirs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv('IDF_VERSION', 'release-v5.5')
    project = tmp_path / 'empty'
    project.mkdir()

    with pytest.raises(SystemExit) as exc:
        merge.run(project)
    assert exc.value.code != 0
