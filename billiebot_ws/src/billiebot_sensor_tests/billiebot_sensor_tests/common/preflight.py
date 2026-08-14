"""One-shot hardware/software preflight capture, run before any ROS node starts.

Plain subprocess.run calls, not launch actions — these are synchronous stdout/stderr
captures with no coordinated process lifecycle, so ExecuteProcess would be the wrong tool.
"""

import json
import subprocess
from pathlib import Path

_PREFLIGHT_COMMANDS = {
    'oakd': [
        ['lsusb'],
        ['lsusb', '-t'],
        ['python3', '-c',
         'import depthai as dai; print(dai.__version__); '
         'print(dai.Device.getAllAvailableDevices())'],
    ],
    'thermal': [
        ['ls', '-l', '/dev/i2c-1'],
        ['i2cdetect', '-y', '1'],
    ],
    'noir': [
        ['rpicam-hello', '--list-cameras'],
        ['python3', '-c',
         'from picamera2 import Picamera2; print(Picamera2.global_camera_info())'],
    ],
    'audio': [
        ['lsusb'],
        ['arecord', '-l'],
        ['arecord', '-L'],
        ['python3', '-c', 'import sounddevice as sd; print(sd.query_devices())'],
        # Recorded, not gated: tflite-runtime is built against the NumPy 1.x ABI, so a
        # host that drifted off the pin explains a classifier that never starts.
        ['python3', '-c',
         'import numpy; print("numpy", numpy.__version__, "(expected 1.26.4)")'],
    ],
}


def _sensor_nano_commands(context: dict) -> list:
    """Sensor Nano preflight, parameterised by the serial port the launch resolved.

    Deliberately separates three different failures that all look like "it didn't work":
    a missing host utility, a device that never enumerated, and a port that enumerated but
    cannot be opened (permissions, or another process holding it). Each command below prints
    the remedy for its own case, because the operator reads this output at the bench.
    """
    port = context.get('sensor_port', '')
    commands = [
        ['lsusb'],
        ['bash', '-lc', 'ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || '
                         'echo "no /dev/ttyUSB* or /dev/ttyACM* device: the Nano did not '
                         'enumerate -- check the USB cable is a data cable and the power LED"'],
        ['bash', '-lc', 'ls -l /dev/serial/by-path/ 2>/dev/null || '
                         'echo "no /dev/serial/by-path entries"'],
        ['bash', '-lc', 'ls -l /dev/serial/by-id/ 2>/dev/null || '
                         'echo "no /dev/serial/by-id entries"'],
        ['python3', '-c',
         'import serial; print("pyserial", serial.__version__)'],
        ['id'],
    ]
    if port:
        commands += [
            ['bash', '-lc',
             f'target=$(readlink -f "{port}" 2>/dev/null); '
             f'if [ -e "{port}" ]; then echo "port {port} -> ${{target:-{port}}}"; '
             f'ls -l "${{target:-{port}}}"; '
             f'[ -r "${{target:-{port}}}" ] && echo readable || echo "NOT READABLE"; '
             f'[ -w "${{target:-{port}}}" ] && echo writable || '
             f'echo "NOT WRITABLE: add the user to the dialout group"; '
             f'else echo "port {port} does not exist"; fi'],
            ['bash', '-lc',
             f'command -v lsof >/dev/null && lsof "$(readlink -f "{port}" 2>/dev/null)" || '
             f'echo "lsof unavailable or port not held by another process"'],
            # Opens the port read-only and reports whether the Nano is actually talking.
            # This is the one check that distinguishes "device present" from "device
            # streaming", which is what the bench actually depends on.
            ['python3', '-c',
             'import sys, serial\n'
             f'port = {port!r}\n'
             'try:\n'
             '    s = serial.Serial(port, 115200, timeout=1)\n'
             'except Exception as exc:\n'
             '    print(f"could not open {port}: {exc}"); sys.exit(1)\n'
             'import time\n'
             'time.sleep(2.5)  # Nano auto-reset on DTR\n'
             's.reset_input_buffer()\n'
             'lines = [s.readline().decode(errors="replace").strip() for _ in range(20)]\n'
             's.close()\n'
             'lines = [ln for ln in lines if ln]\n'
             'print("\\n".join(lines[:10]) or "port opened but no data received")\n'
             'types = {ln[0] for ln in lines if ln}\n'
             'print("record types seen:", sorted(types))\n'
             'print("status record seen:", "S" in types)\n'],
        ]
    else:
        commands.append(
            ['bash', '-lc',
             'echo "no sensor_port supplied to the launch: pass sensor_port:=... so preflight '
             'can verify the device opens and streams"']
        )
    return commands


def _mission_software_commands(context: dict) -> list:
    """Software-only preflight for UT-BAT-02B, which needs no Sensor Nano and no ADC."""
    return [
        ['bash', '-lc', 'ros2 pkg prefix billiebot_mission || '
                         'echo "billiebot_mission is not built/sourced: colcon build and '
                         'source install/setup.bash"'],
        ['bash', '-lc', 'ros2 pkg prefix billiebot_interfaces || '
                         'echo "billiebot_interfaces is not built/sourced"'],
        ['bash', '-lc', 'ros2 pkg prefix nav2_msgs || '
                         'echo "nav2_msgs is missing: mission_controller.py imports '
                         'NavigateToPose at module scope, so the message package must be '
                         'installed (the Nav2 servers do NOT need to be running)"'],
        ['python3', '-c',
         'from billiebot_interfaces.msg import MissionStatus; '
         'print("MissionStatus.SAFE =", MissionStatus.SAFE)'],
    ]


#: Preflights whose command list depends on launch-time context (e.g. the resolved serial
#: port) are callables taking that context dict; the static ones above stay plain lists.
_PREFLIGHT_BUILDERS = {
    'sensor_nano': _sensor_nano_commands,
    'mission_software': _mission_software_commands,
}


def _run_command(cmd: list) -> dict:
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        return {
            'cmd': cmd, 'returncode': proc.returncode,
            'stdout': proc.stdout, 'stderr': proc.stderr,
        }
    except FileNotFoundError as e:
        return {'cmd': cmd, 'returncode': -1, 'stdout': '', 'stderr': str(e)}
    except subprocess.TimeoutExpired as e:
        return {'cmd': cmd, 'returncode': -1, 'stdout': '', 'stderr': f'timeout: {e}'}


def run_preflight(sensor: str, result_dir, context: dict = None) -> dict:
    """Run the preflight command list for `sensor`, append output to console.log, write
    exports/preflight.json, and return the results dict (never raises on command failure —
    a failed preflight command is data, surfaced by the caller's own pass/fail logic).

    `context` supplies launch-resolved values (currently `sensor_port`) to the preflights
    whose commands depend on them; the original four sensors ignore it entirely."""
    builder = _PREFLIGHT_BUILDERS.get(sensor)
    if builder is not None:
        commands = builder(context or {})
    else:
        commands = _PREFLIGHT_COMMANDS.get(sensor, [])
    results = [_run_command(cmd) for cmd in commands]

    with open(result_dir.console_log_path, 'a') as log:
        log.write(f'--- preflight: {sensor} ---\n')
        for r in results:
            log.write(f"$ {' '.join(r['cmd'])}\n")
            log.write(r['stdout'])
            if r['stderr']:
                log.write('\n[stderr]\n' + r['stderr'])
            log.write(f"\n[exit {r['returncode']}]\n\n")

    preflight_path = Path(result_dir.exports_dir) / 'preflight.json'
    with open(preflight_path, 'w') as f:
        json.dump(results, f, indent=2)

    return {'sensor': sensor, 'commands': results}
