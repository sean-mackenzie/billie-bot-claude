"""Pure visualization-preview processing: decimation, depth colorization, compression, and
point-cloud decimation.

This module is the single implementation of the bench suite's *visualization* data path, which
is deliberately separate from the *authoritative* one:

    High-rate/high-volume raw sensor topics are authoritative data products intended for local
    rosbag2 recording and quantitative analysis. Remote visualization should preferentially use
    dedicated downsampled, colorized, compressed, and/or rate-limited `/bench/.../preview` topics.

Nothing produced here is valid for quantitative analysis -- decimation drops pixels, colorization
is lossy and range-clipped, and JPEG is lossy. Analysis code must always read the raw topics.

No ROS, hardware, or cv2 imports: everything below is numpy (plus a lazy `PIL` import inside
`encode_compressed`, so pytest collection and the pure-math tests never require an image codec).
"""

from dataclasses import dataclass

import numpy as np

#: Formats `encode_compressed` accepts. 'raw' means "do not compress" -- the caller publishes a
#: downsampled `sensor_msgs/Image` instead of a `CompressedImage`.
SUPPORTED_FORMATS = ('jpeg', 'png', 'raw')

#: Depth value DepthAI uses for "no measurement". Colorized to `invalid_color` so an operator can
#: never mistake a hole for a real near/far reading.
INVALID_DEPTH_MM = 0


class PreviewConfigError(ValueError):
    pass


def blue_green_red_colormap(normalized: np.ndarray) -> np.ndarray:
    """Hand-rolled blue->green->red colormap (not the literal 'turbo' LUT) mapping values in
    [0,1] to an (H,W,3) uint8 RGB array. Shared by the thermal colorizer and the depth preview so
    the bench has exactly one colormap implementation."""
    t = np.clip(normalized, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    rgb = np.stack([r, g, b], axis=-1)
    return (rgb * 255).astype(np.uint8)


def decimate_stride(source_len: int, target_len: int) -> int:
    """Integer stride that takes `source_len` samples down to at most `target_len`. Never
    upsamples -- a target larger than the source yields stride 1."""
    if source_len <= 0 or target_len <= 0:
        raise PreviewConfigError(
            f'decimate_stride needs positive lengths, got source={source_len} target={target_len}'
        )
    return max(1, -(-source_len // target_len))  # ceil division


def decimate_image(image: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Strided-slice downsample of an (H,W) or (H,W,C) array to approximately target_w x target_h.

    Deliberately nearest-neighbour rather than an averaged/interpolated resize: a strided slice is
    effectively free (no arithmetic, and numpy returns a view), whereas an area resize would cost
    real time inside the acquisition callback for a preview nobody measures. The result is
    therefore *at most* the requested size and generally a little larger, since only integer
    strides are used.
    """
    if image.ndim not in (2, 3):
        raise PreviewConfigError(f'decimate_image expects a 2D or 3D array, got ndim={image.ndim}')
    h, w = image.shape[0], image.shape[1]
    sy = decimate_stride(h, target_h)
    sx = decimate_stride(w, target_w)
    return image[::sy, ::sx]


def colorize_depth(depth_mm: np.ndarray, min_m: float = 0.1, max_m: float = 5.0,
                   invalid_color=(0, 0, 0)) -> np.ndarray:
    """Human-viewable RGB rendering of a 16UC1 millimetre depth image.

    NOT valid for quantitative depth analysis: values are clipped to [min_m, max_m] and quantized
    to 8 bits per channel. UT-OAK-02's accuracy math always reads the raw 16UC1 topic.

    Pixels that are invalid (`INVALID_DEPTH_MM`) or outside the display range are painted
    `invalid_color`, which is outside the blue->green->red ramp's gamut, so holes are visually
    unambiguous rather than blending into the "very near" end of the colormap.
    """
    if depth_mm.ndim != 2:
        raise PreviewConfigError(f'colorize_depth expects a 2D array, got ndim={depth_mm.ndim}')
    if not max_m > min_m:
        raise PreviewConfigError(f'colorize_depth needs max_m > min_m, got {min_m}..{max_m}')

    depth_m = depth_mm.astype(np.float32) / 1000.0
    valid = (depth_mm != INVALID_DEPTH_MM) & (depth_m >= min_m) & (depth_m <= max_m)

    normalized = (depth_m - min_m) / (max_m - min_m)
    normalized = np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0)
    rgb = blue_green_red_colormap(normalized)
    rgb[~valid] = np.asarray(invalid_color, dtype=np.uint8)
    return rgb


def encode_compressed(image: np.ndarray, fmt: str = 'jpeg', quality: int = 70,
                      source_order: str = 'rgb') -> bytes:
    """Encode an (H,W,3) uint8 array to JPEG/PNG bytes for `sensor_msgs/CompressedImage.data`.

    `source_order` is 'bgr' for OAK-D frames (DepthAI's native order) and 'rgb' for anything
    already in display order; the channel swap happens here so no caller has to remember it.
    `PIL` is imported lazily -- the pure decimation/colorization tests must not need a codec, and
    a host missing Pillow degrades to `fmt='raw'` at the node level rather than failing at import.
    """
    fmt = fmt.lower()
    if fmt == 'raw':
        raise PreviewConfigError("encode_compressed cannot encode fmt='raw'; publish an Image")
    if fmt not in SUPPORTED_FORMATS:
        raise PreviewConfigError(f"unsupported preview format '{fmt}' (want one of {SUPPORTED_FORMATS})")
    if image.ndim != 3 or image.shape[2] != 3:
        raise PreviewConfigError(f'encode_compressed expects an (H,W,3) array, got {image.shape}')
    if source_order not in ('rgb', 'bgr'):
        raise PreviewConfigError(f"source_order must be 'rgb' or 'bgr', got '{source_order}'")

    import io

    from PIL import Image as PILImage

    rgb = image[:, :, ::-1] if source_order == 'bgr' else image
    buf = io.BytesIO()
    if fmt == 'jpeg':
        PILImage.fromarray(np.ascontiguousarray(rgb)).save(buf, 'JPEG', quality=int(quality))
    else:
        PILImage.fromarray(np.ascontiguousarray(rgb)).save(buf, 'PNG', compress_level=1)
    return buf.getvalue()


def depth_to_points(depth_mm: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                    stride: int = 1) -> np.ndarray:
    """Back-project a 16UC1 millimetre depth image to an (N,3) float32 XYZ array in metres.

    Shared by the authoritative point cloud (stride 4) and the visualization preview cloud
    (stride 16) so the two differ only by decimation, never by geometry. Zero-depth pixels are
    dropped. Decimating the source depth array is far cheaper than parsing and re-encoding an
    already-serialized PointCloud2, which is why the preview cloud is generated here rather than
    filtered downstream.

    Returns a float32 ndarray, which callers must hand to `create_cloud_xyz32` as-is. Converting
    it to a Python list per frame is what held the authoritative cloud at 0.33 Hz before 990a99a.
    """
    if depth_mm.ndim != 2:
        raise PreviewConfigError(f'depth_to_points expects a 2D array, got ndim={depth_mm.ndim}')
    stride = max(1, int(stride))
    h, w = depth_mm.shape
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    z = depth_mm[0:h:stride, 0:w:stride].astype(np.float32) / 1000.0  # mm -> m
    valid = z > 0
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy
    return np.stack([x[valid], y[valid], z[valid]], axis=-1).astype(np.float32, copy=False)


#: Fraction of the target period a frame may arrive early and still be emitted. Source frames
#: jitter by several milliseconds either side of nominal, so a strict `>= period` comparison
#: rejects roughly a third of a same-rate stream (measured: a 5 Hz limiter passed only 3.5 Hz of
#: a 5 Hz source). The slack lets a preview keep up with a source at its own rate while still
#: capping a genuinely faster one.
RATE_TOLERANCE = 0.1


class RateLimiter:
    """Emit-at-most-N-Hz gate for visualization publishes.

    Lets a preview run slower than its source (e.g. a 2 Hz point-cloud preview off a 5 Hz depth
    stream) without a second ROS timer, and without ever affecting the source's own rate.
    `hz <= 0` disables the limiter (every call emits).

    This is a ceiling on whole source frames, never an interpolator: a 2 Hz limiter on a 5 Hz
    source passes every third frame (~1.67 Hz), not exactly 2 Hz.
    """

    def __init__(self, hz: float, tolerance: float = RATE_TOLERANCE):
        self.hz = float(hz)
        self._period_s = 1.0 / self.hz if self.hz > 0 else 0.0
        self._threshold_s = self._period_s * (1.0 - tolerance)
        self._last_emit_s = None

    def should_emit(self, now_s: float) -> bool:
        if self._period_s <= 0.0:
            return True
        if self._last_emit_s is None or now_s - self._last_emit_s >= self._threshold_s:
            self._last_emit_s = now_s
            return True
        return False


@dataclass
class PreviewConfig:
    """Validated settings for one image/depth preview stream.

    Every field is a visualization knob. None of these may appear in a `thresholds.required` or
    `thresholds.provisional` block -- a preview setting must never be able to change a pass/fail
    verdict.
    """

    width: int = 640
    height: int = 360
    rate_hz: float = 5.0
    quality: int = 70
    format: str = 'jpeg'
    min_m: float = 0.1
    max_m: float = 5.0

    def __post_init__(self):
        self.validate()

    def validate(self) -> 'PreviewConfig':
        if self.width <= 0 or self.height <= 0:
            raise PreviewConfigError(
                f'preview width/height must be positive, got {self.width}x{self.height}'
            )
        if self.rate_hz <= 0:
            raise PreviewConfigError(f'preview rate_hz must be positive, got {self.rate_hz}')
        if not 1 <= self.quality <= 100:
            raise PreviewConfigError(f'preview quality must be in 1..100, got {self.quality}')
        if self.format.lower() not in SUPPORTED_FORMATS:
            raise PreviewConfigError(
                f"unsupported preview format '{self.format}' (want one of {SUPPORTED_FORMATS})"
            )
        if not self.max_m > self.min_m:
            raise PreviewConfigError(
                f'preview depth range needs max_m > min_m, got {self.min_m}..{self.max_m}'
            )
        return self
