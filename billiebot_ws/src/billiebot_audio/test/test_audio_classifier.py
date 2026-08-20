import random
from unittest.mock import MagicMock

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus

from billiebot_audio import xvf3800_control
from billiebot_audio.audio_classifier import AudioClassifier

# XVF3800 USB device resolution and the vendor-control protocol are covered by
# test_xvf3800_control.py, which owns those symbols since the refactor.


def test_fail_loud_on_missing_model_path():
    # model_path defaults to '' -- real mode must exit(1) rather than stay silently alive.
    rclpy.init(args=['--ros-args', '-p', 'mock:=false'])
    try:
        with pytest.raises(SystemExit):
            AudioClassifier()
    finally:
        rclpy.shutdown()


@pytest.fixture()
def mock_node():
    rclpy.init(args=['--ros-args', '-p', 'mock:=true'])
    n = AudioClassifier()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_mock_classify_unaffected(mock_node):
    mock_node.event_pub.publish = MagicMock()
    random.seed(0)
    for _ in range(50):
        mock_node.mock_classify()
    assert mock_node.event_pub.publish.called
    for call in mock_node.event_pub.publish.call_args_list:
        msg = call[0][0]
        assert msg.event_type in (0, 1)  # BARK=0, WHINE=1 -- the only mock-path outputs


def test_mock_mode_performs_no_usb_operations(monkeypatch):
    """Mock mode must never enumerate or touch USB hardware: no power policy, no DoA read,
    no device handle. Any USB access raises, so a regression fails loudly here."""
    def _explode():
        raise AssertionError('mock mode must not enumerate USB devices')

    monkeypatch.setattr(xvf3800_control, 'enumerate_usb_devices', _explode)

    rclpy.init(args=['--ros-args', '-p', 'mock:=true'])
    try:
        node = AudioClassifier()
        try:
            node.event_pub.publish = MagicMock()
            random.seed(0)
            for _ in range(50):
                node.mock_classify()

            assert node._xvf_device is None
            assert node._xvf_policy is None
            assert node._xvf_verify_timer is None
            assert node.status_pub is None
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


class FakeXvfDevice:
    """Stands in for xvf3800_control.Xvf3800Device at the classifier boundary."""

    def __init__(self, policy_result=None, doa=None, doa_error=None):
        self.policy_result = policy_result
        self.doa = doa
        self.doa_error = doa_error
        self.closed = 0

    def read_doa(self):
        if self.doa_error is not None:
            raise self.doa_error
        return self.doa

    def close(self):
        self.closed += 1


def _ok_policy(**overrides):
    values = dict(
        ok=True,
        led_effect=xvf3800_control.LED_EFFECT_DOA,
        gpo=xvf3800_control.GpoState.from_values([0, 0, 1, 0, 0]),
    )
    values.update(overrides)
    return xvf3800_control.PowerPolicyResult(**values)


@pytest.fixture()
def real_node_with_fake_xvf(monkeypatch):
    """A mock-mode node (so no model/sounddevice is needed) with the power policy switched
    on, used to drive the XVF3800 integration paths directly."""
    def _make(device, policy_result):
        monkeypatch.setattr(xvf3800_control, 'open_xvf3800', lambda *a, **k: device)
        monkeypatch.setattr(
            xvf3800_control, 'ensure_billiebot_power_policy', lambda d: policy_result
        )
        rclpy.init(args=['--ros-args', '-p', 'mock:=true'])
        node = AudioClassifier()
        node.xvf3800_power_policy_enabled = True
        return node

    created = []

    def factory(device, policy_result):
        node = _make(device, policy_result)
        created.append(node)
        return node

    yield factory

    for node in created:
        node.destroy_node()
    rclpy.shutdown()


def test_power_policy_applied_records_state(real_node_with_fake_xvf):
    device = FakeXvfDevice()
    node = real_node_with_fake_xvf(device, _ok_policy())

    node._apply_xvf_power_policy(initial=True)

    assert node._xvf_policy.ok is True
    assert node._xvf_last_error == ''
    assert node._xvf_device is device


def test_strict_power_policy_failure_fails_initialization(real_node_with_fake_xvf):
    failed = xvf3800_control.PowerPolicyResult(ok=False, error='X0D33 is 1, expected 0')
    node = real_node_with_fake_xvf(FakeXvfDevice(), failed)
    node.xvf3800_power_policy_strict = True

    with pytest.raises(SystemExit):
        node._apply_xvf_power_policy(initial=True)


def test_non_strict_power_policy_failure_continues(real_node_with_fake_xvf):
    failed = xvf3800_control.PowerPolicyResult(ok=False, error='X0D33 is 1, expected 0')
    node = real_node_with_fake_xvf(FakeXvfDevice(), failed)
    node.xvf3800_power_policy_strict = False

    node._apply_xvf_power_policy(initial=True)   # must not raise

    assert node._xvf_policy.ok is False
    assert 'X0D33' in node._xvf_last_error
    assert node._xvf_device is None   # handle dropped so the next attempt re-resolves


def test_periodic_verification_failure_does_not_raise(real_node_with_fake_xvf):
    """A post-startup control error is recorded and retried, never fatal -- strict or not."""
    failed = xvf3800_control.PowerPolicyResult(ok=False, error='device disconnected')
    node = real_node_with_fake_xvf(FakeXvfDevice(), failed)
    node.xvf3800_power_policy_strict = True

    node._verify_xvf_power_policy()   # must not raise, must not exit

    assert node._xvf_policy.ok is False


def test_get_doa_returns_reading(real_node_with_fake_xvf):
    device = FakeXvfDevice(doa=xvf3800_control.DoaReading(degrees=135, speech_detected=True))
    node = real_node_with_fake_xvf(device, _ok_policy())
    node._xvf_device = device

    assert node._get_doa() == 135.0
    assert node._read_doa_degrees() == 135.0


def test_get_doa_distinguishes_read_failure_from_zero_degrees(real_node_with_fake_xvf):
    """0 deg is a real bearing; a failed read must report None internally even though
    _get_doa still falls back to 0.0 for AudioEvent (DT-AUD-02 gates on this)."""
    device = FakeXvfDevice(doa_error=OSError('pipe error'))
    node = real_node_with_fake_xvf(device, _ok_policy())
    node._xvf_device = device

    assert node._read_doa_degrees() is None
    assert 'pipe error' in node._xvf_last_error
    assert node._xvf_device is None       # invalidated, will re-resolve

    node._xvf_device = FakeXvfDevice(
        doa=xvf3800_control.DoaReading(degrees=0, speech_detected=False)
    )
    assert node._read_doa_degrees() == 0.0   # a genuine 0 deg is not a failure


def test_get_doa_rejects_out_of_range_reading(real_node_with_fake_xvf):
    device = FakeXvfDevice(doa=xvf3800_control.DoaReading(degrees=400, speech_detected=True))
    node = real_node_with_fake_xvf(device, _ok_policy())
    node._xvf_device = device

    assert node._read_doa_degrees() is None
    assert node._get_doa() == 0.0


def _status_keyvalues(node):
    published = []
    node.status_pub = MagicMock()
    node.status_pub.publish.side_effect = published.append
    node.publish_status = True
    node._publish_status(
        cycle_start=1.0,
        inference_duration=0.01,
        result={'energy_db': -30.0, 'passed_energy_gate': True,
                'top_name': 'Bark', 'top_score': 0.9},
    )
    return {kv.key: kv.value for kv in published[0].status[0].values}, published[0]


def test_diagnostics_report_healthy_power_policy(real_node_with_fake_xvf):
    node = real_node_with_fake_xvf(FakeXvfDevice(), _ok_policy())
    node._apply_xvf_power_policy(initial=True)

    kv, msg = _status_keyvalues(node)

    assert kv['xvf3800_power_policy_ok'] == 'true'
    assert kv['xvf3800_led_power_off'] == 'true'
    assert kv['xvf3800_amplifier_disabled'] == 'true'
    assert kv['xvf3800_mic_muted'] == 'false'
    assert kv['xvf3800_led_effect'] == '4'
    assert kv['xvf3800_last_control_error'] == ''
    assert msg.status[0].level == DiagnosticStatus.OK
    # Pre-existing keys are unchanged.
    assert kv['top_label'] == 'Bark'
    assert kv['buffer_overrun_count'] == '0'


def test_diagnostics_report_muted_microphones(real_node_with_fake_xvf):
    muted = _ok_policy(gpo=xvf3800_control.GpoState.from_values([0, 1, 1, 0, 0]))
    node = real_node_with_fake_xvf(FakeXvfDevice(), muted)
    node._apply_xvf_power_policy(initial=True)

    kv, _ = _status_keyvalues(node)

    assert kv['xvf3800_mic_muted'] == 'true'
    assert kv['xvf3800_power_policy_ok'] == 'true'   # mute is observed, not a policy failure


def test_diagnostics_report_unverified_power_policy(real_node_with_fake_xvf):
    failed = xvf3800_control.PowerPolicyResult(ok=False, error='X0D33 is 1, expected 0')
    node = real_node_with_fake_xvf(FakeXvfDevice(), failed)
    node.xvf3800_power_policy_strict = False
    node._apply_xvf_power_policy(initial=True)

    kv, msg = _status_keyvalues(node)

    assert kv['xvf3800_power_policy_ok'] == 'false'
    # Unreadable device state is 'unknown', not a misleading 'false'.
    assert kv['xvf3800_led_power_off'] == 'unknown'
    assert kv['xvf3800_amplifier_disabled'] == 'unknown'
    assert kv['xvf3800_led_effect'] == 'unknown'
    assert 'X0D33' in kv['xvf3800_last_control_error']
    assert msg.status[0].level == DiagnosticStatus.WARN
