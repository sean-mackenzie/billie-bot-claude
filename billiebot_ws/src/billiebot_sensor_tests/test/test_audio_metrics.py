import numpy as np
import pytest

from fixtures.audio_fixtures import EventSample, StatusSample, synthetic_wav

from billiebot_sensor_tests.audio.classifier_scoring import (
    confusion_matrix,
    label_distribution,
    latency_stats,
    precision_recall_f1,
)
from billiebot_sensor_tests.audio.metrics import (
    channel_correlation,
    clipping_fraction,
    dc_offset,
    mains_hum_energy,
    peak_dbfs,
    rms_dbfs,
)
from billiebot_sensor_tests.common.circular_stats import circular_abs_error_deg
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats


def test_silence_has_very_low_rms():
    assert rms_dbfs(synthetic_wav('silence')) < -80


def test_sine_has_expected_rms_and_no_clipping():
    wav = synthetic_wav('sine')
    expected_dbfs = 20 * np.log10(0.5 / np.sqrt(2))
    assert rms_dbfs(wav) == pytest.approx(expected_dbfs, abs=0.5)
    assert clipping_fraction(wav) == pytest.approx(0.0)


def test_clipped_waveform_has_high_clipping_fraction():
    assert clipping_fraction(synthetic_wav('clipped'), threshold=0.99) > 0.1


def test_dc_offset_waveform_detected():
    assert dc_offset(synthetic_wav('dc_offset')) == pytest.approx(0.3, abs=0.01)


def test_white_noise_has_near_zero_dc_offset():
    assert abs(dc_offset(synthetic_wav('white_noise', seed=42))) < 0.02


def test_peak_dbfs_for_clipped_waveform_near_zero():
    assert peak_dbfs(synthetic_wav('clipped')) == pytest.approx(0.0, abs=0.5)


def test_channel_correlation_identical_channels():
    wav = synthetic_wav('sine')
    assert channel_correlation(wav, wav) == pytest.approx(1.0, abs=1e-6)


def test_channel_correlation_uncorrelated_noise_near_zero():
    a = synthetic_wav('white_noise', seed=1)
    b = synthetic_wav('white_noise', seed=2)
    assert abs(channel_correlation(a, b)) < 0.2


def test_mains_hum_energy_detects_50hz_tone():
    sample_rate = 16000
    t = np.arange(sample_rate) / sample_rate
    hum = 0.5 * np.sin(2 * np.pi * 50.0 * t)
    result = mains_hum_energy(hum, sample_rate, freqs=(50.0, 60.0))
    assert result[50.0] > result[60.0]
    assert result[50.0] > 0.5


def test_bark_event_timeline_recall_and_precision():
    predicted = ['BARK'] * 8 + ['LOUD_NOISE'] * 2
    true = ['BARK'] * 10
    metrics = precision_recall_f1(predicted, true, positive_label='BARK')
    assert metrics['recall'] == pytest.approx(0.8)
    assert metrics['precision'] == pytest.approx(1.0)


def test_speech_timeline_maps_to_loud_noise_not_bark():
    predicted = ['LOUD_NOISE'] * 20
    true = ['SPEECH'] * 20
    cm = confusion_matrix(predicted, true, labels=['BARK', 'LOUD_NOISE', 'SPEECH'])
    assert cm['SPEECH']['LOUD_NOISE'] == 20
    assert cm['SPEECH']['BARK'] == 0


def test_inference_timing_and_2hz_processing_cadence():
    status_samples = [
        StatusSample(t_ns=i * int(0.5e9), inference_duration_s=0.05) for i in range(20)
    ]
    stats = compute_rate_stats([s.t_ns for s in status_samples])
    assert stats.mean_hz == pytest.approx(2.0, abs=0.1)
    latencies = latency_stats([s.inference_duration_s for s in status_samples])
    assert latencies['max_s'] <= 0.1


def test_circular_doa_error_spans_0_360_boundary():
    events = [
        EventSample(t_ns=0, event_type=0, confidence=0.9, yamnet_label='Bark',
                    energy_db=-10.0, doa_deg=1.0),
        EventSample(t_ns=1, event_type=0, confidence=0.9, yamnet_label='Bark',
                    energy_db=-10.0, doa_deg=359.0),
    ]
    errors = [circular_abs_error_deg(e.doa_deg, 0.0) for e in events]
    assert all(err == pytest.approx(1.0, abs=1e-6) for err in errors)


def test_label_distribution_sums_to_one():
    dist = label_distribution(['Bark', 'Bark', 'Speech'])
    assert sum(dist.values()) == pytest.approx(1.0)
    assert dist['Bark'] == pytest.approx(2 / 3)
