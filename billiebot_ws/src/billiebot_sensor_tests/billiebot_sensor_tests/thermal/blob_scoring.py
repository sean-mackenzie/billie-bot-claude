"""Pure warm-body blob detection scoring for DT-THM-01. No hardware/ROS imports."""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class BlobSample:
    t_ns: int
    cx: float
    cy: float
    area: int
    max_temp: float
    mean_temp: float
    is_dog_candidate: bool


@dataclass
class ThermalGroundTruthSegment:
    t_start_ns: int
    t_end_ns: int
    dog_present: bool
    distance_m: Optional[float] = None


def associate_blob_to_frame(blobs: List[BlobSample], frame_timestamps_ns: List[int],
                             tolerance_s: float = 0.3) -> dict:
    """Associates each raw frame timestamp with the nearest blob within tolerance.
    Returns {frame_ts_ns: BlobSample or None}."""
    tol_ns = int(tolerance_s * 1e9)
    result = {}
    for ft in frame_timestamps_ns:
        candidates = [b for b in blobs if abs(b.t_ns - ft) <= tol_ns]
        result[ft] = min(candidates, key=lambda b: abs(b.t_ns - ft)) if candidates else None
    return result


def compute_blob_metrics(blobs: List[BlobSample], frame_timestamps_ns: List[int],
                          segments: List[ThermalGroundTruthSegment],
                          area_min: int = 8, temp_min_c: float = 30.0,
                          temp_max_c: float = 40.0,
                          reliable_fraction: float = 0.80) -> dict:
    associated = associate_blob_to_frame(blobs, frame_timestamps_ns)

    def frames_in_segment(seg):
        return [ft for ft in frame_timestamps_ns if seg.t_start_ns <= ft < seg.t_end_ns]

    per_segment_fraction = {}
    for i, seg in enumerate(segments):
        frames = frames_in_segment(seg)
        detected = sum(1 for ft in frames if associated.get(ft) is not None)
        per_segment_fraction[i] = detected / len(frames) if frames else 0.0

    positive_segments = [i for i, s in enumerate(segments) if s.dog_present]
    negative_segments = [i for i, s in enumerate(segments) if not s.dog_present]

    false_positive_frames = sum(
        1 for i in negative_segments for ft in frames_in_segment(segments[i])
        if associated.get(ft) is not None
    )
    total_negative_frames = sum(len(frames_in_segment(segments[i])) for i in negative_segments)
    false_positive_fraction = (
        false_positive_frames / total_negative_frames if total_negative_frames else 0.0
    )

    valid_positive_blobs = [
        b for b in blobs
        if b.is_dog_candidate and b.area >= area_min and temp_min_c <= b.mean_temp <= temp_max_c
    ]
    valid_output_fraction = (
        len(valid_positive_blobs) / len(blobs) if blobs else float('nan')
    )

    reliable_distances = [
        segments[i].distance_m for i in positive_segments
        if segments[i].distance_m is not None and per_segment_fraction[i] >= reliable_fraction
    ]
    max_reliable_distance_m = max(reliable_distances) if reliable_distances else None

    return {
        'per_segment_detection_fraction': per_segment_fraction,
        'false_positive_fraction': false_positive_fraction,
        'valid_output_fraction': valid_output_fraction,
        'max_reliable_distance_m': max_reliable_distance_m,
        'centroid_mean': (
            (float(np.mean([b.cx for b in blobs])), float(np.mean([b.cy for b in blobs])))
            if blobs else None
        ),
        'area_mean': float(np.mean([b.area for b in blobs])) if blobs else None,
        'mean_temp_mean': float(np.mean([b.mean_temp for b in blobs])) if blobs else None,
    }
