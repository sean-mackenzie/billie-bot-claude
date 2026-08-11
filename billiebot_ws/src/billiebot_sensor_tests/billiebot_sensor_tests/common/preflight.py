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


def run_preflight(sensor: str, result_dir) -> dict:
    """Run the preflight command list for `sensor`, append output to console.log, write
    exports/preflight.json, and return the results dict (never raises on command failure —
    a failed preflight command is data, surfaced by the caller's own pass/fail logic)."""
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
