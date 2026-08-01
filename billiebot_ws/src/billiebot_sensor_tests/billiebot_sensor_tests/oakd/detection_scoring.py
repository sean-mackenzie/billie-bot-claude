"""Pure detection-segment scoring for DT-OAK-01. No hardware/ROS imports."""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

__all__ = [
    'DetectionSample', 'GroundTruthSegment',
    'align_detections_to_segments', 'compute_detection_metrics',
]


@dataclass
class DetectionSample:
    t_ns: int
    bbox: tuple  # (x, y, w, h)
    confidence: float
    depth_m: float


@dataclass
class GroundTruthSegment:
    t_start_ns: int
    t_end_ns: int
    dog_present: bool
    distance_m: Optional[float] = None


def align_detections_to_segments(detections: List, segments: List) -> dict:
    """Buckets detections by which ground-truth segment their timestamp falls in.
    Returns {segment_index: [DetectionSample, ...]}."""
    buckets = {i: [] for i in range(len(segments))}
    for det in detections:
        for i, seg in enumerate(segments):
            if seg.t_start_ns <= det.t_ns < seg.t_end_ns:
                buckets[i].append(det)
                break
    return buckets


def compute_detection_metrics(detections: List, segments: List,
                               min_confidence: float = 0.5,
                               reliable_fraction: float = 0.80) -> dict:
    buckets = align_detections_to_segments(detections, segments)

    def confident(d):
        return d.confidence >= min_confidence

    def valid_bbox(d):
        return d.bbox[2] > 0 and d.bbox[3] > 0

    positive_segments = [i for i, s in enumerate(segments) if s.dog_present]
    negative_segments = [i for i, s in enumerate(segments) if not s.dog_present]

    per_segment_fraction = {}
    for i in range(len(segments)):
        dets = buckets[i]
        confident_dets = [d for d in dets if confident(d)]
        per_segment_fraction[i] = (len(confident_dets) / len(dets)) if dets else 0.0

    true_positive_segments = sum(
        1 for i in positive_segments if per_segment_fraction[i] >= reliable_fraction
    )
    recall = (
        (true_positive_segments / len(positive_segments)) if positive_segments else float('nan')
    )

    fp_frame_total = sum(len(buckets[i]) for i in negative_segments)
    fp_confident_total = sum(
        len([d for d in buckets[i] if confident(d)]) for i in negative_segments
    )
    false_positive_fraction = (
        fp_confident_total / fp_frame_total if fp_frame_total else 0.0
    )

    all_confident = [d for dets in buckets.values() for d in dets if confident(d)]
    valid_bbox_fraction = (
        sum(1 for d in all_confident if valid_bbox(d)) / len(all_confident)
        if all_confident else float('nan')
    )

    depth_errors = []
    for i in positive_segments:
        seg = segments[i]
        if seg.distance_m is None:
            continue
        depth_errors.extend(
            abs(d.depth_m - seg.distance_m) for d in buckets[i] if confident(d)
        )
    median_depth_error = float(np.median(depth_errors)) if depth_errors else float('nan')

    reliable_distances = [
        segments[i].distance_m for i in positive_segments
        if segments[i].distance_m is not None and per_segment_fraction[i] >= reliable_fraction
    ]
    min_reliable_distance_m = min(reliable_distances) if reliable_distances else None
    max_reliable_distance_m = max(reliable_distances) if reliable_distances else None

    return {
        'recall': recall,
        'false_positive_fraction': false_positive_fraction,
        'valid_bbox_fraction': valid_bbox_fraction,
        'median_depth_error_m': median_depth_error,
        'min_reliable_distance_m': min_reliable_distance_m,
        'max_reliable_distance_m': max_reliable_distance_m,
        'per_segment_detection_fraction': per_segment_fraction,
    }
