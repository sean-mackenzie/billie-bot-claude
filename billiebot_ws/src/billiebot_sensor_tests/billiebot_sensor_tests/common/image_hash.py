"""Pure repeated-frame detection via a small average-hash. No hardware/ROS imports."""

import numpy as np


def repeated_frame_hash(data: bytes, width: int, height: int, channels: int = 1,
                         grid: int = 8) -> int:
    """Compute a cheap average-hash of an image buffer for repeated/frozen-frame detection.

    Two frames that are pixel-identical (or nearly so) hash to the same integer, which is
    all that is needed to flag a stuck/repeated camera stream — this is not a general
    perceptual-similarity hash.
    """
    expected_len = width * height * channels
    if len(data) < expected_len:
        raise ValueError(
            f'buffer too short: got {len(data)} bytes, expected {expected_len} '
            f'for {width}x{height}x{channels}'
        )
    arr = np.frombuffer(data, dtype=np.uint8, count=expected_len).reshape(height, width, channels)
    gray = arr.mean(axis=2) if channels > 1 else arr[:, :, 0]

    # Block-average downsample to a grid x grid grid (grid must evenly divide via clipping).
    row_edges = np.linspace(0, height, grid + 1).astype(int)
    col_edges = np.linspace(0, width, grid + 1).astype(int)
    cells = np.zeros((grid, grid), dtype=np.float64)
    for i in range(grid):
        r0, r1 = row_edges[i], max(row_edges[i] + 1, row_edges[i + 1])
        for j in range(grid):
            c0, c1 = col_edges[j], max(col_edges[j] + 1, col_edges[j + 1])
            cells[i, j] = gray[r0:r1, c0:c1].mean()

    threshold = cells.mean()
    bits = (cells > threshold).astype(np.uint64).flatten()
    h = 0
    for bit in bits:
        h = (h << 1) | int(bit)
    return h
