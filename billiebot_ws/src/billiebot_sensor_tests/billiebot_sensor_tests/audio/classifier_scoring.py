"""Pure classifier scoring for DT-AUD-01/02. No hardware/ROS imports."""

from collections import Counter
from typing import List

import numpy as np


def confusion_matrix(predicted_labels: List[str], true_labels: List[str],
                      labels: List[str]) -> dict:
    matrix = {t: {p: 0 for p in labels} for t in labels}
    for pred, true in zip(predicted_labels, true_labels):
        if true in matrix and pred in matrix[true]:
            matrix[true][pred] += 1
    return matrix


def precision_recall_f1(predicted_labels: List[str], true_labels: List[str],
                         positive_label: str) -> dict:
    tp = sum(1 for p, t in zip(predicted_labels, true_labels)
             if p == positive_label and t == positive_label)
    fp = sum(1 for p, t in zip(predicted_labels, true_labels)
             if p == positive_label and t != positive_label)
    fn = sum(1 for p, t in zip(predicted_labels, true_labels)
             if p != positive_label and t == positive_label)
    precision = (tp / (tp + fp)) if (tp + fp) else float('nan')
    recall = (tp / (tp + fn)) if (tp + fn) else float('nan')
    f1 = (
        2 * precision * recall / (precision + recall)
        if precision == precision and recall == recall and (precision + recall) > 0
        else float('nan')
    )
    return {'precision': precision, 'recall': recall, 'f1': f1, 'tp': tp, 'fp': fp, 'fn': fn}


def latency_stats(latencies_s: List[float]) -> dict:
    values = np.asarray(latencies_s, dtype=np.float64)
    if values.size == 0:
        return {'mean_s': float('nan'), 'median_s': float('nan'), 'max_s': float('nan'),
                'p95_s': float('nan')}
    return {
        'mean_s': float(np.mean(values)),
        'median_s': float(np.median(values)),
        'max_s': float(np.max(values)),
        'p95_s': float(np.percentile(values, 95)),
    }


def label_distribution(labels: List[str]) -> dict:
    counts = Counter(labels)
    total = sum(counts.values()) or 1
    return {label: count / total for label, count in counts.items()}
