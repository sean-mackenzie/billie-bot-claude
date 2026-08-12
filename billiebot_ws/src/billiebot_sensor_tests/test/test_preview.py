"""Unit tests for the pure visualization-preview processing in common/preview.py.

Everything here is synthetic: no OAK-D, no ROS, no bag.
"""

import io

import numpy as np
import pytest

from fixtures.preview_fixtures import (
    synthetic_bgr,
    synthetic_depth_mm,
    synthetic_depth_plane_mm,
    synthetic_red_bgr,
)

from billiebot_sensor_tests.common.preview import (
    INVALID_DEPTH_MM,
    PreviewConfig,
    PreviewConfigError,
    RateLimiter,
    colorize_depth,
    decimate_image,
    decimate_stride,
    depth_to_points,
    encode_compressed,
)

_INTRINSICS = dict(fx=450.0, fy=450.0, cx=320.0, cy=200.0)


# --- decimation ------------------------------------------------------------------------


def test_decimate_1080p_to_640x360_hits_target_budget():
    small = decimate_image(synthetic_bgr(), 640, 360)
    assert small.shape[1] <= 640 and small.shape[0] <= 360
    assert small.shape == (360, 640, 3)  # 1920/3, 1080/3 -- exact integer stride


def test_decimate_non_integer_ratio_stays_within_budget():
    small = decimate_image(synthetic_bgr(1280, 720), 640, 360)
    assert small.shape[1] <= 640 and small.shape[0] <= 360


def test_decimate_never_upscales():
    src = synthetic_bgr(320, 180)
    assert decimate_image(src, 1920, 1080).shape == src.shape


def test_decimate_preserves_channel_count_and_dtype():
    small = decimate_image(synthetic_bgr(640, 480), 320, 240)
    assert small.shape[2] == 3 and small.dtype == np.uint8


def test_decimate_works_on_2d_depth():
    small = decimate_image(synthetic_depth_mm(640, 400), 320, 200)
    assert small.ndim == 2 and small.shape == (200, 320)


def test_decimate_stride_rejects_nonpositive():
    with pytest.raises(PreviewConfigError):
        decimate_stride(0, 100)


def test_decimate_rejects_wrong_rank():
    with pytest.raises(PreviewConfigError):
        decimate_image(np.zeros((2, 2, 3, 3), dtype=np.uint8), 1, 1)


# --- compression -----------------------------------------------------------------------


def _decode(data: bytes):
    from PIL import Image as PILImage
    return np.asarray(PILImage.open(io.BytesIO(data)).convert('RGB'))


def test_jpeg_encode_round_trips_to_same_dimensions():
    small = decimate_image(synthetic_bgr(), 640, 360)
    decoded = _decode(encode_compressed(small, 'jpeg', 70, source_order='bgr'))
    assert decoded.shape == (360, 640, 3)


def test_jpeg_is_far_smaller_than_the_raw_payload():
    small = decimate_image(synthetic_bgr(), 640, 360)
    encoded = encode_compressed(small, 'jpeg', 70, source_order='bgr')
    # The whole point of the change: the Foxglove payload must be orders of magnitude below
    # the 6.2 MB raw frame it is derived from.
    assert len(encoded) < small.nbytes / 5
    assert len(encoded) < synthetic_bgr().nbytes / 50


def test_lower_jpeg_quality_yields_smaller_payload():
    small = decimate_image(synthetic_bgr(), 640, 360)
    low = encode_compressed(small, 'jpeg', 40, source_order='bgr')
    high = encode_compressed(small, 'jpeg', 90, source_order='bgr')
    assert len(low) < len(high)


def test_png_encode_round_trips_losslessly():
    src = synthetic_red_bgr()
    decoded = _decode(encode_compressed(src, 'png', source_order='bgr'))
    assert np.array_equal(decoded[:, :, 0], np.full(src.shape[:2], 255, dtype=np.uint8))


def test_bgr_source_order_survives_encoding_as_red_not_blue():
    decoded = _decode(encode_compressed(synthetic_red_bgr(), 'jpeg', 95, source_order='bgr'))
    r, g, b = decoded[..., 0].mean(), decoded[..., 1].mean(), decoded[..., 2].mean()
    assert r > 200 and g < 60 and b < 60


def test_rgb_source_order_is_not_swapped():
    rgb = np.zeros((32, 32, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    decoded = _decode(encode_compressed(rgb, 'jpeg', 95, source_order='rgb'))
    assert decoded[..., 0].mean() > 200 and decoded[..., 2].mean() < 60


def test_encode_rejects_raw_and_unknown_formats():
    src = synthetic_red_bgr()
    with pytest.raises(PreviewConfigError):
        encode_compressed(src, 'raw')
    with pytest.raises(PreviewConfigError):
        encode_compressed(src, 'webp')


def test_encode_rejects_non_three_channel_input():
    with pytest.raises(PreviewConfigError):
        encode_compressed(np.zeros((8, 8), dtype=np.uint8), 'jpeg')


def test_encode_rejects_bad_source_order():
    with pytest.raises(PreviewConfigError):
        encode_compressed(synthetic_red_bgr(), 'jpeg', source_order='gbr')


# --- depth colorization ----------------------------------------------------------------


def test_colorize_depth_output_shape_and_dtype():
    rgb = colorize_depth(synthetic_depth_mm(64, 32))
    assert rgb.shape == (32, 64, 3) and rgb.dtype == np.uint8


def test_colorize_depth_varies_monotonically_across_the_range():
    """The ramp must actually distinguish near from far -- a colormap that collapsed to one
    colour would look plausible in Foxglove while conveying nothing."""
    depth = synthetic_depth_mm(256, 8, near_mm=200, far_mm=4800)
    rgb = colorize_depth(depth, 0.1, 5.0)
    columns = rgb[0]
    assert len({tuple(c) for c in columns}) > 50


def test_invalid_depth_is_painted_the_invalid_colour():
    depth = synthetic_depth_plane_mm(64, 32, 2000)
    depth[0:8, 0:8] = INVALID_DEPTH_MM
    rgb = colorize_depth(depth, 0.1, 5.0)
    assert np.all(rgb[0:8, 0:8] == 0)
    assert not np.all(rgb[16:, 16:] == 0)


def test_out_of_range_depth_is_treated_as_invalid():
    depth = synthetic_depth_plane_mm(16, 16, 9000)  # 9.0 m, beyond a 5.0 m display range
    assert np.all(colorize_depth(depth, 0.1, 5.0) == 0)


def test_display_range_rescales_the_same_frame():
    depth = synthetic_depth_plane_mm(16, 16, 2000)
    near_range = colorize_depth(depth, 0.1, 2.5)
    wide_range = colorize_depth(depth, 0.1, 10.0)
    assert not np.array_equal(near_range, wide_range)


def test_custom_invalid_colour_is_honoured():
    depth = synthetic_depth_plane_mm(8, 8, 0)
    rgb = colorize_depth(depth, 0.1, 5.0, invalid_color=(255, 0, 255))
    assert np.all(rgb == np.array([255, 0, 255], dtype=np.uint8))


def test_colorize_depth_rejects_inverted_range():
    with pytest.raises(PreviewConfigError):
        colorize_depth(synthetic_depth_plane_mm(4, 4), min_m=5.0, max_m=0.1)


# --- point-cloud decimation ------------------------------------------------------------


def test_preview_stride_yields_roughly_one_sixteenth_the_points():
    depth = synthetic_depth_plane_mm(640, 400, 2000)
    authoritative = depth_to_points(depth, stride=4, **_INTRINSICS)
    preview = depth_to_points(depth, stride=16, **_INTRINSICS)
    ratio = len(authoritative) / len(preview)
    assert 14 <= ratio <= 18


def test_decimation_does_not_change_the_geometry_of_the_points_it_keeps():
    """The preview cloud must be a subset of the authoritative one, not a different projection --
    otherwise a Foxglove eyeball check would not correspond to the recorded data."""
    depth = synthetic_depth_plane_mm(64, 64, 2000)
    coarse = depth_to_points(depth, stride=8, **_INTRINSICS)
    fine = depth_to_points(depth, stride=4, **_INTRINSICS)
    fine_set = {tuple(np.round(p, 6)) for p in fine}
    assert all(tuple(np.round(p, 6)) in fine_set for p in coarse)


def test_invalid_depth_pixels_are_dropped_from_the_cloud():
    depth = synthetic_depth_plane_mm(32, 32, 2000)
    depth[0:16, :] = INVALID_DEPTH_MM
    points = depth_to_points(depth, stride=1, **_INTRINSICS)
    assert len(points) == 16 * 32
    assert np.all(points[:, 2] > 0)


def test_points_are_metres_from_millimetre_input():
    points = depth_to_points(synthetic_depth_plane_mm(8, 8, 2000), stride=1, **_INTRINSICS)
    assert np.allclose(points[:, 2], 2.0)


def test_depth_to_points_rejects_wrong_rank():
    with pytest.raises(PreviewConfigError):
        depth_to_points(np.zeros((4, 4, 4), dtype=np.uint16), **_INTRINSICS)


# --- rate limiting ---------------------------------------------------------------------


def test_rate_limiter_caps_a_5hz_source_at_the_configured_rate():
    """A 2 Hz limiter on a 5 Hz source emits every 3rd frame (~1.67 Hz), not every 2.5th -- the
    gate can only pass whole source frames. It is a ceiling, never an interpolator, which is why
    the point-cloud preview is specified as 1-2 Hz rather than exactly 2 Hz."""
    limiter = RateLimiter(2.0)
    emitted = sum(limiter.should_emit(i * 0.2) for i in range(11))  # 11 ticks over 2.0 s
    assert emitted == 4
    assert emitted <= 2.0 * 2.0 + 1  # never above the configured rate over the window


def test_rate_limiter_first_call_always_emits():
    assert RateLimiter(1.0).should_emit(1234.5)


def test_rate_limiter_suppresses_within_the_period():
    limiter = RateLimiter(1.0)
    assert limiter.should_emit(0.0)
    assert not limiter.should_emit(0.5)
    assert limiter.should_emit(1.0)


def test_same_rate_source_is_not_thinned_by_jitter():
    """Regression for a measured defect: a strict `>= period` comparison dropped ~30% of a 5 Hz
    source through a 5 Hz limiter, because frame arrivals jitter either side of nominal. A
    preview asked to run at its source's rate must keep up with it."""
    rng = np.random.default_rng(7)
    limiter = RateLimiter(5.0)
    t = 0.0
    emitted = 0
    for _ in range(100):
        emitted += limiter.should_emit(t)
        t += 0.2 + rng.normal(0, 0.01)  # 5 Hz with realistic jitter
    assert emitted >= 95


def test_tolerance_still_caps_a_faster_source():
    limiter = RateLimiter(2.0)
    emitted = sum(limiter.should_emit(i * 0.02) for i in range(500))  # 50 Hz source over 10 s
    assert emitted <= 23  # ~2 Hz over 10 s, slack included


def test_non_positive_rate_disables_limiting():
    limiter = RateLimiter(0.0)
    assert all(limiter.should_emit(t) for t in (0.0, 0.0, 0.0))


# --- configuration validation ----------------------------------------------------------


def test_default_preview_config_is_valid():
    assert PreviewConfig().validate() is not None


@pytest.mark.parametrize('kwargs', [
    {'width': 0},
    {'height': -1},
    {'rate_hz': 0.0},
    {'quality': 0},
    {'quality': 101},
    {'format': 'tiff'},
    {'min_m': 5.0, 'max_m': 1.0},
    {'min_m': 1.0, 'max_m': 1.0},
])
def test_preview_config_rejects_invalid_fields(kwargs):
    with pytest.raises(PreviewConfigError):
        PreviewConfig(**kwargs)


@pytest.mark.parametrize('fmt', ['jpeg', 'png', 'raw'])
def test_supported_formats_validate(fmt):
    assert PreviewConfig(format=fmt).format == fmt
