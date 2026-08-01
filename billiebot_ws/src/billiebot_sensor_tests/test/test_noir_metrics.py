import pytest

from fixtures.noir_fixtures import synthetic_chart

from billiebot_sensor_tests.common.image_hash import repeated_frame_hash
from billiebot_sensor_tests.noir.metrics import (
    clipping_fraction,
    contrast_to_noise_ratio,
    laplacian_variance,
    temporal_stability,
)


def test_sharp_chart_has_higher_laplacian_variance_than_blurred():
    sharp = synthetic_chart(sharp=True)[:, :, 0]
    blurred = synthetic_chart(sharp=False)[:, :, 0]
    assert laplacian_variance(sharp) > laplacian_variance(blurred)


def test_low_contrast_reduces_cnr():
    # A tiny amount of noise keeps std > 0 for both patches so the ratio stays finite --
    # otherwise a perfectly flat synthetic patch has zero noise and CNR is trivially inf.
    normal = synthetic_chart(contrast=1.0, noise_std=3.0, seed=1)
    low = synthetic_chart(contrast=0.1, noise_std=3.0, seed=1)
    cnr_normal = contrast_to_noise_ratio(normal[:, 32:, 0], normal[:, :32, 0])
    cnr_low = contrast_to_noise_ratio(low[:, 32:, 0], low[:, :32, 0])
    assert cnr_normal > cnr_low


def test_clipped_image_has_nonzero_clipping_fraction():
    clipped = synthetic_chart(clip=True)
    assert clipping_fraction(clipped[:, :, 0]) > 0.0


def test_unclipped_image_has_near_zero_clipping_fraction():
    normal = synthetic_chart(clip=False, contrast=0.5)
    assert clipping_fraction(normal[:, :, 0]) < 0.05


def test_noisy_sequence_has_worse_temporal_stability_than_stable():
    stable_values = [100.0, 100.5, 99.8, 100.2, 100.1]
    noisy_values = [80.0, 130.0, 60.0, 150.0, 90.0]
    assert temporal_stability(noisy_values) > temporal_stability(stable_values)


def test_repeated_frame_detected_for_identical_charts():
    a = synthetic_chart(seed=1)
    b = synthetic_chart(seed=1)
    h_a = repeated_frame_hash(a.tobytes(), a.shape[1], a.shape[0], channels=3)
    h_b = repeated_frame_hash(b.tobytes(), b.shape[1], b.shape[0], channels=3)
    assert h_a == h_b


def test_different_frames_not_flagged_as_repeated():
    # synthetic_chart's half-dark/half-light macro layout is identical across noise/contrast
    # variants, which an 8x8 average-hash intentionally treats as "the same coarse frame" --
    # that's the correct behavior for a repeated-frame detector, not a bug. Pixel-inverting
    # the chart is a genuinely different frame under any reasonable hash.
    a = synthetic_chart(seed=1)
    b = 255 - a
    h_a = repeated_frame_hash(a.tobytes(), a.shape[1], a.shape[0], channels=3)
    h_b = repeated_frame_hash(b.tobytes(), b.shape[1], b.shape[0], channels=3)
    assert h_a != h_b


def test_roi_based_contrast_target_white_vs_black_patch():
    chart = synthetic_chart()
    black_patch = chart[:, :16, 0]
    white_patch = chart[:, -16:, 0]
    assert contrast_to_noise_ratio(white_patch, black_patch) > 10
