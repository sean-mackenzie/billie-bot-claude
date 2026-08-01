import numpy as np

from billiebot_audio.audio_ring_buffer import AudioRingBuffer


def test_write_read_wraparound():
    buf = AudioRingBuffer(capacity_samples=10, channels=1)
    buf.write(np.arange(5, dtype=np.float32))
    result = buf.read_last(5)
    assert np.allclose(result[:, 0], [0, 1, 2, 3, 4])

    buf.write(np.arange(5, 12, dtype=np.float32))  # 7 more samples, forces a wraparound
    result = buf.read_last(10)
    assert np.allclose(result[:, 0], np.arange(2, 12))


def test_read_last_returns_none_when_not_full_enough():
    buf = AudioRingBuffer(capacity_samples=10, channels=1)
    buf.write(np.arange(3, dtype=np.float32))
    assert buf.read_last(5) is None


def test_overrun_count_increments_on_status_flag():
    buf = AudioRingBuffer(capacity_samples=10, channels=1)
    assert buf.overrun_count == 0
    buf.write(np.zeros(2, dtype=np.float32), overflowed=True)
    assert buf.overrun_count == 1
    buf.write(np.zeros(2, dtype=np.float32), overflowed=False)
    assert buf.overrun_count == 1


def test_write_larger_than_capacity_keeps_most_recent():
    buf = AudioRingBuffer(capacity_samples=5, channels=1)
    buf.write(np.arange(20, dtype=np.float32))
    result = buf.read_last(5)
    assert np.allclose(result[:, 0], np.arange(15, 20))
