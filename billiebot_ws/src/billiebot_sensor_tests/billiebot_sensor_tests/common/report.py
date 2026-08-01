"""Shared Markdown report renderer used by every analyze_*/score_* CLI, so report structure
is consistent across all 11 bench tests."""

import json
from pathlib import Path
from typing import Iterable


def render_report_md(test_id: str, metrics: dict, thresholds_used: dict, pass_fail: dict,
                      plots: Iterable = ()) -> str:
    """
    thresholds_used: {criterion_name: {'value':, 'threshold':, 'tier': 'required'|'provisional'}}
    pass_fail: {criterion_name: bool, ..., 'overall': bool}
    """
    overall = pass_fail.get('overall', all(v for k, v in pass_fail.items() if k != 'overall'))

    lines = [f'# Bench Report: {test_id}', '']
    lines.append(f"**Overall result:** {'PASS' if overall else 'FAIL'}")
    lines.append('')
    lines.append('## Pass/fail criteria')
    lines.append('')
    lines.append('| Criterion | Value | Threshold | Tier | Result |')
    lines.append('|---|---|---|---|---|')
    for name, info in thresholds_used.items():
        value = info.get('value')
        threshold = info.get('threshold')
        tier = info.get('tier', 'required')
        ok = pass_fail.get(name)
        result = 'PASS' if ok is True else ('FAIL' if ok is False else '—')
        lines.append(f'| {name} | {value} | {threshold} | {tier} | {result} |')
    lines.append('')
    lines.append('## Metrics')
    lines.append('')
    lines.append('```json')
    lines.append(json.dumps(metrics, indent=2, default=str))
    lines.append('```')
    if plots:
        lines.append('')
        lines.append('## Plots')
        lines.append('')
        for p in plots:
            lines.append(f'- `{Path(p).name}`')
    lines.append('')
    return '\n'.join(lines)


def write_metrics_json(path, metrics: dict, pass_fail: dict) -> None:
    payload = dict(metrics)
    payload['pass'] = bool(pass_fail.get('overall', all(
        v for k, v in pass_fail.items() if k != 'overall'
    )))
    payload['pass_fail'] = pass_fail
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2, default=str)


def write_metrics_csv(path, metrics: dict) -> None:
    import csv

    flat = _flatten(metrics)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['key', 'value'])
        for key, value in flat.items():
            writer.writerow([key, value])


def _flatten(d: dict, prefix: str = '') -> dict:
    out = {}
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            out.update(_flatten(v, key))
        else:
            out[key] = v
    return out
