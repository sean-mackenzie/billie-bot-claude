"""Owns the manifest.yaml lifecycle: written once at launch start, updated/finalized at
launch shutdown. Captures host/OS/ROS-distro/git identity so every result is reproducible."""

import os
import platform
import socket
import subprocess
from datetime import datetime, timezone

import yaml

from billiebot_sensor_tests.common.result_dir import BenchResultDir


def _run(cmd, cwd=None) -> str:
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=10, cwd=cwd)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def _git_info(repo_dir=None):
    sha = _run(['git', 'rev-parse', 'HEAD'], cwd=repo_dir) or 'unknown'
    dirty = bool(_run(['git', 'status', '--porcelain'], cwd=repo_dir))
    return sha, dirty


class ManifestWriter:
    def __init__(self, result_dir: BenchResultDir):
        self.result_dir = result_dir
        self._data = {}

    @classmethod
    def load(cls, result_dir: BenchResultDir) -> 'ManifestWriter':
        """Reloads a manifest.yaml written by a previous ManifestWriter instance (e.g. by
        write_initial() in a launch bootstrap action) so a later, separate action instance
        (e.g. a shutdown handler) can finalize() it without sharing Python object state."""
        writer = cls(result_dir)
        if result_dir.manifest_path.exists():
            with open(result_dir.manifest_path, 'r') as f:
                writer._data = yaml.safe_load(f) or {}
        return writer

    def write_initial(self, test_id: str, sensor_model: str, sensor_serial: str,
                       launch_command: str, parameters: dict, config_file: str,
                       repo_dir: str = None) -> None:
        sha, dirty = _git_info(repo_dir)
        self._data = {
            'test_id': test_id,
            'start_time_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'host': socket.gethostname(),
            'host_os': platform.platform(),
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
            'repository_commit': sha,
            'repository_dirty': dirty,
            'sensor_model': sensor_model,
            'sensor_serial': sensor_serial or '',
            'launch_command': launch_command,
            'config_file': config_file,
            'parameters': parameters or {},
            'ground_truth': {},
            'operator_notes': '',
            'software_versions': {},
            'detected_hardware_interfaces': {},
        }
        self._write()

    def record_ground_truth(self, ground_truth: dict) -> None:
        self._data.setdefault('ground_truth', {}).update(ground_truth)
        self._write()

    def record_detected_hardware(self, interfaces: dict) -> None:
        self._data.setdefault('detected_hardware_interfaces', {}).update(interfaces)
        self._write()

    def record_software_versions(self, versions: dict) -> None:
        self._data.setdefault('software_versions', {}).update(versions)
        self._write()

    def finalize(self, exit_code: int, operator_notes: str = None) -> None:
        self._data['end_time_utc'] = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
        self._data['exit_code'] = exit_code
        if operator_notes:
            self._data['operator_notes'] = operator_notes
        self._write()

    def _write(self) -> None:
        with open(self.result_dir.manifest_path, 'w') as f:
            yaml.safe_dump(self._data, f, sort_keys=False)
