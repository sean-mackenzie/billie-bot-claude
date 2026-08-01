"""Fixed-size circular audio buffer decoupling capture (a PortAudio callback thread) from
periodic inference (the ROS timer thread).

This is what makes true ~2 Hz processing possible in audio_classifier.py's
continuous_capture path: the old code blocked the ROS timer on a 0.975 s sd.rec() call
every tick; this buffer lets capture run continuously on its own PortAudio-managed thread
while the timer only ever does a fast, non-blocking read of the last N samples.
"""

import threading

import numpy as np


class AudioRingBuffer:
    def __init__(self, capacity_samples: int, channels: int = 1):
        self._capacity = int(capacity_samples)
        self._channels = channels
        self._buffer = np.zeros((self._capacity, channels), dtype=np.float32)
        self._write_pos = 0
        self._filled = 0
        self._overrun_count = 0
        self._lock = threading.Lock()

    def write(self, samples: np.ndarray, overflowed: bool = False) -> None:
        """Called from the audio callback thread. Never blocks on I/O."""
        if samples.ndim == 1:
            samples = samples[:, None]
        n = samples.shape[0]
        with self._lock:
            if overflowed:
                self._overrun_count += 1
            if n >= self._capacity:
                self._buffer[:] = samples[-self._capacity:]
                self._write_pos = 0
                self._filled = self._capacity
                return
            end = self._write_pos + n
            if end <= self._capacity:
                self._buffer[self._write_pos:end] = samples
            else:
                first_part = self._capacity - self._write_pos
                self._buffer[self._write_pos:] = samples[:first_part]
                self._buffer[: end - self._capacity] = samples[first_part:]
            self._write_pos = end % self._capacity
            self._filled = min(self._filled + n, self._capacity)

    def read_last(self, n_samples: int):
        """Called from the ROS timer thread. Returns the most recent n_samples samples as
        a (n_samples, channels) array, or None if the buffer does not yet hold that many
        samples (still warming up)."""
        with self._lock:
            if self._filled < n_samples:
                return None
            start = (self._write_pos - n_samples) % self._capacity
            if start + n_samples <= self._capacity:
                return self._buffer[start:start + n_samples].copy()
            first_part = self._capacity - start
            return np.concatenate([
                self._buffer[start:], self._buffer[: n_samples - first_part]
            ])

    @property
    def overrun_count(self) -> int:
        with self._lock:
            return self._overrun_count
