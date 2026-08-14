"""Shared launch-time actions imported by all 7 bench launch files.

Consolidating this logic here means no launch file hand-rolls result-directory creation,
manifest lifecycle, bag recording, or Foxglove startup — every launch file wires the same
building blocks together with different topic lists / production-node overrides.
"""

import shlex
import sys
from pathlib import Path

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

from billiebot_sensor_tests.common.manifest import ManifestWriter
from billiebot_sensor_tests.common.preflight import run_preflight
from billiebot_sensor_tests.common.result_dir import BenchResultDir


def declare_common_bench_args(default_duration_sec: str = '60') -> list:
    """Common launch args every bench launch file supports. `results_dir` is deliberately
    mandatory (no default) — a bench run must always land in an explicit, timestamped
    directory rather than silently reusing/overwriting a default location."""
    return [
        DeclareLaunchArgument('results_dir', description='root output directory (required)'),
        DeclareLaunchArgument('duration_sec', default_value=default_duration_sec,
                               description='0 disables the automatic shutdown timer'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('start_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('sensor_serial', default_value=''),
        DeclareLaunchArgument('fail_on_missing_device', default_value='true'),
    ]


def manifest_bootstrap_action(test_id: str, sensor: str, sensor_model: str = '',
                               preflight_context_args: tuple = ()) -> OpaqueFunction:
    """Creates the result directory, writes the initial manifest.yaml, and runs the
    sensor's hardware/software preflight — all before any Node in the launch file starts,
    since this is a plain top-level action and launch executes top-level actions in order.

    `preflight_context_args` names launch arguments whose resolved values are handed to the
    preflight (e.g. `('sensor_port',)` so the Sensor Nano preflight can check that specific
    device rather than guessing). The values are also recorded in manifest.yaml's
    `parameters`, so a result always states which port it was captured from."""

    def _bootstrap(context, *args, **kwargs):
        results_dir = LaunchConfiguration('results_dir').perform(context)
        config_file = LaunchConfiguration('config_file').perform(context)
        sensor_serial = LaunchConfiguration('sensor_serial').perform(context)

        preflight_context = {
            name: LaunchConfiguration(name).perform(context)
            for name in preflight_context_args
        }

        result_dir = BenchResultDir.create(results_dir)
        manifest = ManifestWriter(result_dir)
        manifest.write_initial(
            test_id=_resolve_test_id(test_id, context),
            sensor_model=sensor_model,
            sensor_serial=sensor_serial,
            launch_command=shlex.join(sys.argv),
            parameters=dict(preflight_context),
            config_file=config_file,
        )
        run_preflight(sensor, result_dir, context=preflight_context)
        return []

    return OpaqueFunction(function=_bootstrap)


def _resolve_test_id(test_id, context) -> str:
    """Resolves `test_id` if it is a launch Substitution (has .perform), else returns it
    unchanged. Lets callers pass either a plain str test_id or a runtime-derived one (e.g.
    a PythonExpression picking between two IDs that share one launch file, such as
    UT-OAK-01/UT-OAK-02)."""
    return test_id.perform(context) if hasattr(test_id, 'perform') else test_id


def finalize_on_shutdown_action() -> RegisterEventHandler:
    """Folds per-node logs into console.log and finalizes manifest.yaml on launch shutdown.
    Runs identically whether the launch was started by run_sensor_test or a bare
    `ros2 launch ...` invocation, since OnShutdown fires regardless of how launch exits.
    Needs no test_id -- ManifestWriter.load() reloads the value manifest_bootstrap_action
    already wrote to disk."""

    def _finalize(event, context):
        results_dir = LaunchConfiguration('results_dir').perform(context)
        result_dir = BenchResultDir(results_dir)

        _fold_ros_logs(result_dir)

        manifest = ManifestWriter.load(result_dir)
        manifest.finalize(exit_code=0)
        return []

    return RegisterEventHandler(OnShutdown(on_shutdown=_finalize))


def _fold_ros_logs(result_dir: BenchResultDir) -> None:
    log_dir = result_dir.path / 'log'
    if not log_dir.is_dir():
        return
    try:
        with open(result_dir.console_log_path, 'a') as out:
            out.write('--- folded ROS node logs ---\n')
            for log_file in sorted(log_dir.rglob('*.log')):
                out.write(f'\n=== {log_file.relative_to(log_dir)} ===\n')
                out.write(log_file.read_text(errors='replace'))
    except OSError:
        pass


def duration_shutdown_action() -> list:
    """Shuts the launch down `duration_sec` after this action runs, unless duration_sec is
    the literal string '0' (used by Type 2 launches, which run until the operator stops
    them since they cover many discrete ground-truth segments in one session)."""
    return [
        TimerAction(
            period=LaunchConfiguration('duration_sec'),
            actions=[EmitEvent(event=Shutdown(reason='duration_sec elapsed'))],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('duration_sec'), "' != '0'"])
            ),
        ),
    ]


def foxglove_bridge_action() -> Node:
    return Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': LaunchConfiguration('foxglove_port')}],
        condition=IfCondition(LaunchConfiguration('start_foxglove')),
        output='screen',
    )


def record_bag_action(topics: list) -> list:
    """`ros2 bag record` as a launch-time ExecuteProcess — see plan assumption 3 for why
    this is preferred over a hand-rolled rosbag2_py.Recorder inside a custom node. SIGINT
    on launch Shutdown lets `ros2 bag record` flush metadata.yaml cleanly (its documented
    behavior), so no special shutdown ordering is needed here."""
    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', PathJoinSubstitution([LaunchConfiguration('results_dir'), 'bag']),
                '--storage', 'sqlite3',
                *topics,
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('record_bag')),
        ),
    ]


def replicate_production_node(package: str, executable: str, config_file: str,
                               extra_params: dict, name: str = None) -> Node:
    """billiebot_sensor_tests never edits billiebot_bringup or the production packages'
    own launch files — every detection-bench launch inlines its own Node(...) replicating
    the corresponding bringup rung, merging bench-only param overrides on top of the
    production config file. When every bench-only param is at its documented sentinel
    default, this Node call is behaviorally identical to the bringup rung.

    `name` defaults to `executable`, which is right for the ament_python packages whose
    executables are bare names. It must be given for an ament_cmake package that installs
    its entry point with a file extension: billiebot_mission's executable is literally
    `mission_controller.py`, and a ROS node name cannot contain a dot — mission.launch.py
    itself pairs `executable='mission_controller.py'` with `name='mission_controller'` for
    the same reason. The name also has to match the key in the production params file
    (`mission.yaml` is keyed `mission_controller:` with no wildcard), or none of the
    production parameters would load."""
    return Node(
        package=package,
        executable=executable,
        name=name or executable,
        parameters=[config_file, extra_params],
        output='screen',
    )
