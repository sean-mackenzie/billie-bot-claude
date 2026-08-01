import numpy as np
import pytest

from fixtures.oakd_fixtures import DetectionSample, GroundTruthSegment, synthetic_plane

from billiebot_sensor_tests.oakd.detection_scoring import compute_detection_metrics
from billiebot_sensor_tests.oakd.metrics import (
    fit_plane_svd,
    fraction_within_band,
    percentiles,
    plane_normal_angle_deg,
    point_to_plane_rmse,
    robust_std_mad,
    valid_pixel_fraction,
)


def test_ideal_flat_plane_zero_bias_and_rmse():
    points = synthetic_plane(2.0)
    normal, centroid = fit_plane_svd(points)
    assert point_to_plane_rmse(points, normal, centroid) < 1e-9
    assert np.median(points[:, 2]) == pytest.approx(2.0, abs=1e-9)


def test_tilted_plane_has_expected_normal_angle():
    points = synthetic_plane(2.0, tilt_deg=15.0)
    normal, _centroid = fit_plane_svd(points)
    assert plane_normal_angle_deg(normal) == pytest.approx(15.0, abs=0.5)


def test_biased_plane_median_bias():
    points = synthetic_plane(2.0, bias_m=0.15)
    bias = np.median(points[:, 2]) - 2.0
    assert bias == pytest.approx(0.15, abs=1e-9)


def test_gaussian_noise_robust_std_matches_input_std():
    points = synthetic_plane(2.0, noise_std_m=0.03, seed=2)
    assert robust_std_mad(points[:, 2]) == pytest.approx(0.03, rel=0.3)


def test_invalid_depth_holes_reduce_valid_pixel_fraction():
    points = synthetic_plane(2.0, hole_fraction=0.2, seed=3)
    frac = valid_pixel_fraction(points[:, 2], min_m=0.1, max_m=10.0)
    assert 0.75 <= frac <= 0.85


def test_outliers_reduce_fraction_within_band():
    points = synthetic_plane(2.0, outlier_fraction=0.1, seed=4)
    frac = fraction_within_band(points[:, 2], ground_truth_m=2.0, band_m=0.20)
    assert frac < 0.95


def test_insufficient_valid_pixels_flagged():
    points = synthetic_plane(2.0, hole_fraction=0.5, seed=5)
    assert valid_pixel_fraction(points[:, 2]) < 0.90


def test_percentiles_bracket_median():
    points = synthetic_plane(2.0, noise_std_m=0.05, seed=6)
    p = percentiles(points[:, 2], ps=(5, 50, 95))
    assert p[5] <= p[50] <= p[95]


def test_fit_plane_svd_requires_at_least_three_points():
    with pytest.raises(ValueError):
        fit_plane_svd(np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]))


def test_detection_scoring_true_positive_segment():
    detections = [
        DetectionSample(t_ns=i * int(0.2e9), bbox=(10, 10, 50, 50), confidence=0.9, depth_m=2.0)
        for i in range(10)
    ]
    segments = [
        GroundTruthSegment(t_start_ns=0, t_end_ns=int(2.0e9), dog_present=True, distance_m=2.0)
    ]
    metrics = compute_detection_metrics(detections, segments, min_confidence=0.5)
    assert metrics['recall'] == pytest.approx(1.0)
    assert metrics['valid_bbox_fraction'] == pytest.approx(1.0)
    assert metrics['median_depth_error_m'] == pytest.approx(0.0)


def test_detection_scoring_false_positive_segment():
    detections = [
        DetectionSample(t_ns=i * int(0.2e9), bbox=(10, 10, 50, 50), confidence=0.9, depth_m=2.0)
        for i in range(5)
    ]
    segments = [GroundTruthSegment(t_start_ns=0, t_end_ns=int(1.0e9), dog_present=False)]
    metrics = compute_detection_metrics(detections, segments)
    assert metrics['false_positive_fraction'] == pytest.approx(1.0)


def test_detection_scoring_missed_detection_segment():
    segments = [
        GroundTruthSegment(t_start_ns=0, t_end_ns=int(2.0e9), dog_present=True, distance_m=2.0)
    ]
    metrics = compute_detection_metrics([], segments)
    assert metrics['recall'] == pytest.approx(0.0)


def test_detection_scoring_flags_zero_bbox_as_invalid():
    detections = [DetectionSample(t_ns=0, bbox=(0, 0, 0, 0), confidence=0.9, depth_m=2.0)]
    segments = [
        GroundTruthSegment(t_start_ns=0, t_end_ns=int(1.0e9), dog_present=True, distance_m=2.0)
    ]
    metrics = compute_detection_metrics(detections, segments)
    assert metrics['valid_bbox_fraction'] == pytest.approx(0.0)
