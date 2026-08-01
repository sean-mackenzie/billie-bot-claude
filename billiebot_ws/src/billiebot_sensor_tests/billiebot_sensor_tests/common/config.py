"""Loader/accessor for config/sensor_bench.yaml.

Distinguishes required (system-requirement) thresholds from provisional (engineering-gate)
thresholds so that no acceptance limit is ever a bare literal in analysis code, and so a
provisional threshold is always visibly flagged as such wherever it gates a result.
"""

import warnings

import yaml

_MISSING = object()


class ConfigError(Exception):
    pass


class BenchConfig:
    def __init__(self, data: dict, logger=None):
        self._data = data
        self._logger = logger

    def _lookup(self, dotted_key: str):
        node = self._data
        for part in dotted_key.split('.'):
            if not isinstance(node, dict) or part not in node:
                return _MISSING
            node = node[part]
        return node

    def _warn(self, message: str) -> None:
        if self._logger is not None:
            self._logger.warning(message)
        else:
            warnings.warn(message)

    def required(self, dotted_key: str):
        """A system-requirement threshold. Missing keys are a hard configuration error."""
        val = self._lookup(dotted_key)
        if val is _MISSING:
            raise ConfigError(f"required config key '{dotted_key}' is missing")
        return val

    def provisional(self, dotted_key: str, default=None):
        """An engineering-gate threshold. Always logs that a provisional value is in play,
        so reports never silently look like a hard system requirement."""
        val = self._lookup(dotted_key)
        if val is _MISSING:
            self._warn(
                f"PROVISIONAL threshold '{dotted_key}' not set in config; using default {default}"
            )
            return default
        self._warn(
            f"PROVISIONAL threshold '{dotted_key}' = {val} is gating this result "
            '(engineering gate, not a system requirement)'
        )
        return val

    def get(self, dotted_key: str, default=None):
        """A warning-tier or informational value. Never gates pass/fail by itself."""
        val = self._lookup(dotted_key)
        return default if val is _MISSING else val

    def as_dict(self) -> dict:
        return self._data


def load_bench_config(path: str) -> BenchConfig:
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except OSError as e:
        raise ConfigError(f"failed to read config file '{path}': {e}") from e
    except yaml.YAMLError as e:
        raise ConfigError(f"failed to parse config file '{path}': {e}") from e
    if not isinstance(data, dict):
        raise ConfigError(f"config file '{path}' did not parse to a mapping")
    return BenchConfig(data)
