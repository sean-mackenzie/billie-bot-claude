#!/usr/bin/env python3
"""ros2 run billiebot_sensor_tests run_sensor_test --test-id UT-OAK-01 --results-dir ...

Constructs an in-process launch.LaunchService (not a subprocess wrapping `ros2 launch`) so
this orchestrator and a bare `ros2 launch billiebot_sensor_tests <file>.launch.py ...`
invocation go through exactly the same launch-file code path. Pass/fail always comes from
finalize_check.aggregate_exit_code (reading metrics.json from disk), never from
LaunchService.run()'s return code.
"""

import argparse
import importlib
import os
import sys
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from billiebot_sensor_tests.orchestrate.finalize_check import aggregate_exit_code
from billiebot_sensor_tests.orchestrate.test_registry import TEST_REGISTRY


def _default_results_dir(test_id: str) -> str:
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return os.path.expanduser(f'~/billiebot_test_results/{test_id}_{stamp}')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Run a BillieBot sensor bench test by ID and score it.'
    )
    parser.add_argument('--test-id', required=True, choices=sorted(TEST_REGISTRY.keys()))
    parser.add_argument('--results-dir', default=None)
    parser.add_argument('--duration-sec', type=int, default=None)
    parser.add_argument('--config-file', default='')
    parser.add_argument('--sensor-serial', default='')
    parser.add_argument('--record-bag', default='true', choices=['true', 'false'])
    parser.add_argument('--start-foxglove', default='true', choices=['true', 'false'])
    parser.add_argument('--profile', default=None,
                         help='Override the test spec default analysis profile')
    parser.add_argument('--extra-launch-arg', action='append', default=[],
                         help='key:=value, repeatable, forwarded to the launch file')
    args = parser.parse_args(argv)

    spec = TEST_REGISTRY[args.test_id]
    results_dir = args.results_dir or _default_results_dir(args.test_id)
    duration_sec = (
        args.duration_sec if args.duration_sec is not None else spec.default_duration_sec
    )
    config_file = args.config_file or os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )

    share = get_package_share_directory('billiebot_sensor_tests')
    launch_file_path = os.path.join(share, 'launch', spec.launch_file)

    launch_arguments = {
        'results_dir': results_dir,
        'duration_sec': str(duration_sec),
        'record_bag': args.record_bag,
        'start_foxglove': args.start_foxglove,
        'config_file': config_file,
        'sensor_serial': args.sensor_serial,
    }
    for pair in args.extra_launch_arg:
        key, _, value = pair.partition(':=')
        launch_arguments[key] = value

    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments=launch_arguments.items(),
        )
    ])
    launch_service = LaunchService()
    launch_service.include_launch_description(ld)
    launch_service.run()  # blocks until Shutdown; return code is NOT authoritative

    analysis_exit = 0
    if spec.analysis_module is not None:
        module = importlib.import_module(spec.analysis_module)
        analysis_argv = ['--results-dir', results_dir, '--config-file', config_file]
        profile = args.profile or spec.profile
        if profile:
            analysis_argv += ['--profile', profile]
        analysis_exit = module.main(analysis_argv)

    exit_code = aggregate_exit_code(results_dir, extra_exit_code=analysis_exit)
    print(
        f'[run_sensor_test] {args.test_id}: '
        f'{"PASS" if exit_code == 0 else "FAIL"} -- results in {results_dir}'
    )
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
