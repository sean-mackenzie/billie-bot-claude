"""Authoritative pass/fail exit-code aggregation.

Reads metrics.json (and, if present, the rate monitor's topic_rate_monitor.json) written
to <results_dir>/, never a process/launch return code. This is what makes "run via
run_sensor_test" and "run via a bare `ros2 launch` + separate offline analyze_*/score_*
CLI call, days later" produce identical verdicts.
"""

import json

from billiebot_sensor_tests.common.result_dir import BenchResultDir


def aggregate_exit_code(results_dir, extra_exit_code: int = 0) -> int:
    result_dir = BenchResultDir(results_dir)
    ok = extra_exit_code == 0

    if result_dir.metrics_json_path.exists():
        with open(result_dir.metrics_json_path, 'r') as f:
            metrics = json.load(f)
        ok = ok and bool(metrics.get('pass', False))
    else:
        ok = False

    rate_monitor_path = result_dir.exports_dir / 'topic_rate_monitor.json'
    if rate_monitor_path.exists():
        with open(rate_monitor_path, 'r') as f:
            rate_result = json.load(f)
        ok = ok and bool(rate_result.get('pass', True))

    return 0 if ok else 1
