"""Owns the <results_dir>/{manifest.yaml,console.log,bag/,exports/,plots/,metrics.json,
metrics.csv,report.md} layout and its creation/completeness lifecycle."""

from pathlib import Path


class BenchResultDir:
    # 'bag' is deliberately NOT pre-created here: `ros2 bag record -o <dir>` refuses to
    # write into a directory that already exists (even if empty), so it must create its
    # own bag/ subdirectory at recording time.
    SUBDIRS = ('exports', 'plots')

    def __init__(self, path):
        self.path = Path(path)

    @classmethod
    def create(cls, path) -> 'BenchResultDir':
        p = Path(path)
        if p.exists() and any(p.iterdir()):
            raise FileExistsError(
                f"results_dir '{p}' already exists and is not empty — "
                'use a new timestamped directory rather than overwriting prior results'
            )
        p.mkdir(parents=True, exist_ok=True)
        result = cls(p)
        for sub in cls.SUBDIRS:
            (p / sub).mkdir(parents=True, exist_ok=True)
        return result

    @property
    def bag_dir(self) -> Path:
        return self.path / 'bag'

    @property
    def exports_dir(self) -> Path:
        return self.path / 'exports'

    @property
    def plots_dir(self) -> Path:
        return self.path / 'plots'

    @property
    def manifest_path(self) -> Path:
        return self.path / 'manifest.yaml'

    @property
    def console_log_path(self) -> Path:
        return self.path / 'console.log'

    @property
    def metrics_json_path(self) -> Path:
        return self.path / 'metrics.json'

    @property
    def metrics_csv_path(self) -> Path:
        return self.path / 'metrics.csv'

    @property
    def report_path(self) -> Path:
        return self.path / 'report.md'

    def is_complete(self) -> bool:
        """True once every required artifact of a finished run exists."""
        return (
            self.manifest_path.exists()
            and self.metrics_json_path.exists()
            and self.report_path.exists()
        )
