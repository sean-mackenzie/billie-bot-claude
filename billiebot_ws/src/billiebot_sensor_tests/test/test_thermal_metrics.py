import numpy as np
import pytest

from fixtures.thermal_fixtures import synthetic_thermal_frame

from billiebot_sensor_tests.thermal.blob_scoring import (
    BlobSample,
    ThermalGroundTruthSegment,
    compute_blob_metrics,
)
from billiebot_sensor_tests.thermal.metrics import (
    contrast_to_noise_ratio,
    drift,
    pooled_cnr,
    target_background_bias,
    temporal_noise,
)


def test_ambient_frame_uniform_no_warm_pixels():
    frame = synthetic_thermal_frame(ambient_c=22.0)
    assert np.all(frame == pytest.approx(22.0))


def test_warm_roi_detected_as_higher_temp():
    frame = synthetic_thermal_frame(ambient_c=22.0, blobs=[(16, 12, 4, 35.0)])
    reshaped = frame.reshape(24, 32)
    assert reshaped[12, 16] == pytest.approx(35.0)
    assert reshaped[0, 0] == pytest.approx(22.0)


def test_known_target_background_bias():
    target = synthetic_thermal_frame(ambient_c=35.0)
    background = synthetic_thermal_frame(ambient_c=22.0)
    result = target_background_bias(target, background, ref_target_c=34.5, ref_background_c=22.0)
    assert result['target_bias_c'] == pytest.approx(0.5, abs=1e-6)
    assert result['background_bias_c'] == pytest.approx(0.0, abs=1e-6)


def test_cnr_and_pooled_cnr_positive_for_separated_distributions():
    target = synthetic_thermal_frame(ambient_c=35.0, noise_std_c=0.2, seed=1)
    background = synthetic_thermal_frame(ambient_c=22.0, noise_std_c=0.2, seed=2)
    assert contrast_to_noise_ratio(target, background) > 5
    assert pooled_cnr(target, background) > 5


def test_temporal_noise_sequence_matches_injected_std():
    frames = np.stack([
        synthetic_thermal_frame(ambient_c=22.0, noise_std_c=0.3, seed=s) for s in range(30)
    ])
    noise = temporal_noise(frames)
    assert noise.shape == (768,)
    assert np.mean(noise) == pytest.approx(0.3, rel=0.4)


def test_nan_pixels_must_be_filtered_before_bias_computation():
    frame = synthetic_thermal_frame(ambient_c=22.0, nan_pixels=[0, 1, 2])
    assert np.isnan(frame[0])
    # target_background_bias uses np.median (not nanmedian) -- NaN propagates unless the
    # caller filters first, which is the documented, intended contract.
    with_nans = target_background_bias(frame, frame, ref_target_c=22.0, ref_background_c=22.0)
    assert np.isnan(with_nans['target_bias_c'])
    finite = frame[np.isfinite(frame)]
    clean = target_background_bias(finite, finite, ref_target_c=22.0, ref_background_c=22.0)
    assert clean['target_bias_c'] == pytest.approx(0.0, abs=1e-6)


def test_multiple_disconnected_warm_regions_pool_into_one_global_area():
    # thermal_node.py's production algorithm is pure global thresholding with no
    # connected-component clustering -- this fixture documents that limitation rather
    # than silently "fixing" it in the test.
    frame = synthetic_thermal_frame(
        ambient_c=22.0, blobs=[(4, 4, 2, 35.0), (28, 20, 2, 36.0)],
    )
    warm_pixels = frame[(frame >= 30.0) & (frame <= 40.0)]
    assert warm_pixels.size > 8


def test_area_boundary_just_below_and_above_production_threshold():
    min_blob_area = 8
    below = synthetic_thermal_frame(ambient_c=22.0, blobs=[(16, 12, 1, 35.0)])
    above = synthetic_thermal_frame(ambient_c=22.0, blobs=[(16, 12, 3, 35.0)])
    n_below = int(np.sum((below >= 30.0) & (below <= 40.0)))
    n_above = int(np.sum((above >= 30.0) & (above <= 40.0)))
    assert n_below < min_blob_area
    assert n_above >= min_blob_area


def test_blob_scoring_detection_fraction_and_valid_output():
    blobs = [
        BlobSample(t_ns=i * int(0.25e9), cx=16, cy=12, area=20, max_temp=36, mean_temp=35,
                   is_dog_candidate=True)
        for i in range(4)
    ]
    frame_ts = [i * int(0.25e9) for i in range(4)]
    segments = [
        ThermalGroundTruthSegment(t_start_ns=0, t_end_ns=int(1.0e9), dog_present=True,
                                   distance_m=1.0)
    ]
    metrics = compute_blob_metrics(blobs, frame_ts, segments)
    assert metrics['per_segment_detection_fraction'][0] == pytest.approx(1.0)
    assert metrics['valid_output_fraction'] == pytest.approx(1.0)


def test_blob_scoring_empty_baseline_no_false_positives():
    frame_ts = [i * int(0.25e9) for i in range(4)]
    segments = [ThermalGroundTruthSegment(t_start_ns=0, t_end_ns=int(1.0e9), dog_present=False)]
    metrics = compute_blob_metrics([], frame_ts, segments)
    assert metrics['false_positive_fraction'] == pytest.approx(0.0)


def test_drift_over_stationary_capture():
    values = [22.0, 22.1, 22.05, 22.3, 22.2]
    assert drift(values) == pytest.approx(0.2, abs=1e-6)
