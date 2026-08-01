import numpy as np
import pytest

from fixtures.common_fixtures import (
    gapped_timestamps,
    ideal_timestamps,
    jittered_timestamps,
    nonmonotonic_timestamps,
)

from billiebot_sensor_tests.common.circular_stats import (
    circular_abs_error_deg,
    circular_mean_deg,
    circular_std_deg,
)
from billiebot_sensor_tests.common.config import BenchConfig, ConfigError, load_bench_config
from billiebot_sensor_tests.common.image_hash import repeated_frame_hash
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats


def test_ideal_rate():
    ts = ideal_timestamps(50, 5.0)
    stats = compute_rate_stats(ts)
    assert stats.monotonic
    assert stats.mean_hz == pytest.approx(5.0, abs=1e-6)
    assert stats.median_hz == pytest.approx(5.0, abs=1e-6)
    assert stats.max_gap_s == pytest.approx(0.2, abs=1e-6)


def test_jittered_rate_within_limits():
    ts = jittered_timestamps(200, 5.0, jitter_std_s=0.01, seed=1)
    stats = compute_rate_stats(ts)
    assert stats.monotonic
    assert stats.mean_hz == pytest.approx(5.0, abs=0.5)
    assert stats.period_std_s < 0.05


def test_nonmonotonic_timestamps_detected():
    ts = nonmonotonic_timestamps(10, 5.0, swap_index=3)
    stats = compute_rate_stats(ts)
    assert not stats.monotonic


def test_missing_data_empty_sequence():
    stats = compute_rate_stats([])
    assert stats.count == 0
    assert stats.mean_hz == 0.0
    assert stats.monotonic  # vacuously true


def test_single_sample_sequence():
    stats = compute_rate_stats([12345])
    assert stats.count == 1
    assert stats.monotonic


def test_large_inter_message_gap_detected():
    ts = gapped_timestamps(10, 5.0, gap_index=5, gap_s=2.0)
    stats = compute_rate_stats(ts)
    assert stats.max_gap_s >= 2.0


def test_circular_error_spans_0_360_boundary():
    assert circular_abs_error_deg(1.0, 359.0) == pytest.approx(2.0)
    assert circular_abs_error_deg(359.0, 1.0) == pytest.approx(2.0)
    assert circular_abs_error_deg(0.0, 180.0) == pytest.approx(180.0)


def test_circular_mean_and_std():
    mean = circular_mean_deg([350.0, 10.0])
    assert mean == pytest.approx(0.0, abs=1e-6) or mean == pytest.approx(360.0, abs=1e-6)
    std = circular_std_deg([0.0, 0.0, 0.0])
    assert std == pytest.approx(0.0, abs=1e-6)


def test_repeated_frame_hash_detects_identical_frames():
    img = (np.ones((16, 16), dtype=np.uint8) * 120).tobytes()
    assert repeated_frame_hash(img, 16, 16, 1) == repeated_frame_hash(img, 16, 16, 1)


def test_repeated_frame_hash_differs_for_different_frames():
    rng = np.random.default_rng(0)
    img_a = rng.integers(0, 255, (16, 16), dtype=np.uint8).tobytes()
    img_b = rng.integers(0, 255, (16, 16), dtype=np.uint8).tobytes()
    assert repeated_frame_hash(img_a, 16, 16, 1) != repeated_frame_hash(img_b, 16, 16, 1)


def test_config_required_missing_raises():
    cfg = BenchConfig({'a': {'b': 1}})
    assert cfg.required('a.b') == 1
    with pytest.raises(ConfigError):
        cfg.required('a.missing')


def test_config_provisional_uses_default_when_unset():
    cfg = BenchConfig({})
    assert cfg.provisional('x.y', default=0.5) == 0.5


def test_malformed_config_raises(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('not: [valid: yaml: here')
    with pytest.raises(ConfigError):
        load_bench_config(str(bad))


def test_config_file_not_a_mapping_raises(tmp_path):
    not_a_map = tmp_path / 'list.yaml'
    not_a_map.write_text('- 1\n- 2\n')
    with pytest.raises(ConfigError):
        load_bench_config(str(not_a_map))


def test_config_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError):
        load_bench_config(str(tmp_path / 'does_not_exist.yaml'))
