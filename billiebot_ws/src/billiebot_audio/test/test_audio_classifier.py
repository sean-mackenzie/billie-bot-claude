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


# --- Reconnect / power-policy-restore coverage -----------------------------------------
#
# These drive the REAL xvf3800_control.Xvf3800Device and the REAL
# ensure_billiebot_power_policy() against a wire-level USB fake, so they prove the actual
# GPO traffic and its ordering rather than trusting a stubbed policy.

_CTRL_IN = 0x80


class ScriptedXvfUsb:
    """Wire-level XVF3800 fake: answers LED_EFFECT/GPO/DOA reads from mutable state, applies
    GPO and LED_EFFECT writes, and records every transfer in order so a test can prove that
    the power policy was enforced *before* DoA was read."""

    def __init__(self, led_effect=xvf3800_control.LED_EFFECT_DOA,
                 x0d30=0, x0d31=1, x0d33=0, doa=0):
        self.led_effect = led_effect
        self.pins = {11: 0, 30: x0d30, 31: x0d31, 33: x0d33, 39: 0}
        self.doa = doa
        self.fail_doa = False
        self.log = []   # ('read', cmd) | ('write', cmd, payload_tuple), in call order

    def ctrl_transfer(self, bmRequestType, bRequest, wValue, wIndex, data_or_length,
                      timeout=None):
        if bmRequestType & _CTRL_IN:
            command_id = wValue & ~_CTRL_IN
            self.log.append(('read', command_id))
            if command_id == xvf3800_control.CMD_LED_EFFECT:
                return bytearray([0, self.led_effect])
            if command_id == xvf3800_control.CMD_GPO_READ_VALUES:
                return bytearray([0] + [self.pins[p]
                                        for p in xvf3800_control.GPO_PIN_ORDER])
            if command_id == xvf3800_control.CMD_DOA_VALUE:
                if self.fail_doa:
                    raise OSError('pipe error')
                return bytearray([0, self.doa & 0xFF, (self.doa >> 8) & 0xFF, 1, 0])
            raise AssertionError(f'unscripted read: command {command_id}')

        payload = tuple(bytearray(data_or_length))
        self.log.append(('write', wValue, payload))
        if wValue == xvf3800_control.CMD_GPO_WRITE_VALUE:
            self.pins[payload[0]] = payload[1]
        elif wValue == xvf3800_control.CMD_LED_EFFECT:
            self.led_effect = payload[0]
        return len(payload)

    # --- log helpers --------------------------------------------------------------------

    def gpo_writes(self):
        return [entry[2] for entry in self.log
                if entry[0] == 'write' and entry[1] == xvf3800_control.CMD_GPO_WRITE_VALUE]

    def index_of_gpo_write(self, pin):
        for i, entry in enumerate(self.log):
            if (entry[0] == 'write' and entry[1] == xvf3800_control.CMD_GPO_WRITE_VALUE
                    and entry[2][0] == pin):
                return i
        return None

    def index_of_doa_read(self):
        for i, entry in enumerate(self.log):
            if entry[0] == 'read' and entry[1] == xvf3800_control.CMD_DOA_VALUE:
                return i
        return None


@pytest.fixture()
def node_with_scripted_usb(monkeypatch):
    """Builds a node whose open_xvf3800() hands out the given ScriptedXvfUsb fakes in order,
    each wrapped in a real Xvf3800Device. Returns (node, opened_list)."""
    def factory(*usb_devices):
        pending = list(usb_devices)
        opened = []

        def fake_open(product_substring, *args, **kwargs):
            if not pending:
                raise xvf3800_control.Xvf3800NotFoundError('no more fake devices')
            usb = pending.pop(0)
            opened.append(usb)
            return xvf3800_control.Xvf3800Device(usb, sleep=lambda _s: None)

        monkeypatch.setattr(xvf3800_control, 'open_xvf3800', fake_open)

        rclpy.init(args=['--ros-args', '-p', 'mock:=true'])
        node = AudioClassifier()
        node.xvf3800_power_policy_enabled = True
        node.xvf3800_power_policy_strict = True
        created.append(node)
        return node, opened

    created = []
    yield factory
    for node in created:
        node.destroy_node()
    rclpy.shutdown()


def test_reconnect_restores_power_policy_before_doa(node_with_scripted_usb):
    """The regression this whole revision exists for: after a USB reset the replacement
    device comes back at firmware defaults, and the policy must be re-applied BEFORE the new
    device is used for DoA -- not up to 30 s later when the periodic timer happens to fire."""
    healthy = ScriptedXvfUsb(doa=90)
    # The array as it comes back from a reset: rainbow LEDs, amplifier on, ring powered.
    rebooted = ScriptedXvfUsb(led_effect=2, x0d31=0, x0d33=1, doa=200)

    node, opened = node_with_scripted_usb(healthy, rebooted)
    timer_calls = []
    node._verify_xvf_power_policy = lambda: timer_calls.append(1)

    # 1. Startup: device #1 is already compliant, so the policy performs zero writes.
    node._apply_xvf_power_policy(initial=True)
    assert opened == [healthy]
    assert healthy.gpo_writes() == []
    assert node._xvf_policy.ok is True

    # 2. The device goes away mid-read -> handle invalidated.
    healthy.fail_doa = True
    assert node._read_doa_degrees() is None
    assert node._xvf_device is None
    assert 'pipe error' in node._xvf_last_error

    # 3. Next DoA request reacquires the device.
    doa = node._read_doa_degrees()

    # 4a. A new device was opened...
    assert opened == [healthy, rebooted]
    # ...and the policy was actually enforced on it.
    assert rebooted.led_effect == xvf3800_control.LED_EFFECT_DOA
    assert rebooted.pins[31] == 1     # amplifier disabled
    assert rebooted.pins[33] == 0     # LED ring unpowered
    # 4b. X0D30 still never written, even on the recovery path.
    assert rebooted.pins[30] == 0
    assert all(pin != xvf3800_control.PIN_MIC_MUTE for pin, _ in rebooted.gpo_writes())

    # 4c. Ordering proof at the wire level: the LED-ring power-down transfer precedes the
    # DOA_VALUE transfer, so the ring was dark before the device was ever trusted for DoA.
    ring_off_at = rebooted.index_of_gpo_write(xvf3800_control.PIN_LED_RING_POWER)
    amp_off_at = rebooted.index_of_gpo_write(xvf3800_control.PIN_AMPLIFIER_ENABLE)
    doa_read_at = rebooted.index_of_doa_read()
    assert ring_off_at is not None and amp_off_at is not None and doa_read_at is not None
    assert ring_off_at < doa_read_at
    assert amp_off_at < doa_read_at

    # 5. The DoA read then succeeds against the recovered device.
    assert doa == 200.0
    assert node._xvf_policy.ok is True
    assert node._xvf_device is not None
    assert node._xvf_last_error == ''

    # 6. None of this needed the 30 s periodic timer.
    assert timer_calls == []


def test_reconnect_to_unverifiable_device_returns_none_without_crashing(
        node_with_scripted_usb):
    """If the reacquired device will not accept the policy, it must not be used for DoA --
    but a post-startup failure must never take the node down, strict or not."""
    class StubbornUsb(ScriptedXvfUsb):
        """Accepts GPO writes but silently refuses to apply them, so readback mismatches."""

        def ctrl_transfer(self, bmRequestType, bRequest, wValue, wIndex, data_or_length,
                          timeout=None):
            if (not (bmRequestType & _CTRL_IN)
                    and wValue == xvf3800_control.CMD_GPO_WRITE_VALUE):
                self.log.append(('write', wValue, tuple(bytearray(data_or_length))))
                return len(data_or_length)   # acknowledged, not applied
            return super().ctrl_transfer(bmRequestType, bRequest, wValue, wIndex,
                                         data_or_length, timeout)

    healthy = ScriptedXvfUsb(doa=90)
    stubborn = StubbornUsb(x0d33=1, doa=200)

    node, opened = node_with_scripted_usb(healthy, stubborn)
    node._apply_xvf_power_policy(initial=True)

    healthy.fail_doa = True
    assert node._read_doa_degrees() is None

    # Reacquisition happens, policy fails verification, DoA is refused.
    assert node._read_doa_degrees() is None          # no exception, no SystemExit
    assert opened == [healthy, stubborn]
    assert stubborn.index_of_doa_read() is None      # never trusted for DoA
    assert node._xvf_policy.ok is False
    assert 'X0D33' in node._xvf_last_error
    # _get_doa still honors its float contract for AudioEvent.
    assert node._get_doa() == 0.0


def test_successful_doa_clears_stale_error(node_with_scripted_usb):
    """A recovered control path must not keep advertising an old DoA failure."""
    usb = ScriptedXvfUsb(doa=42)
    node, _ = node_with_scripted_usb(usb)
    node._apply_xvf_power_policy(initial=True)

    usb.fail_doa = True
    assert node._read_doa_degrees() is None
    assert 'pipe error' in node._xvf_last_error

    # Recover in place: the handle was invalidated, so reacquisition re-applies the policy.
    usb.fail_doa = False
    node._xvf_device = xvf3800_control.Xvf3800Device(usb, sleep=lambda _s: None)

    assert node._read_doa_degrees() == 42.0
    assert node._xvf_last_error == ''


def test_successful_doa_does_not_clear_active_policy_failure(real_node_with_fake_xvf):
    """DoA success proves the control path works; it does not prove the LED ring is off, so
    a live power-policy failure must survive it."""
    failed = xvf3800_control.PowerPolicyResult(ok=False, error='X0D33 is 1, expected 0')
    device = FakeXvfDevice(doa=xvf3800_control.DoaReading(degrees=42, speech_detected=True))
    node = real_node_with_fake_xvf(device, failed)
    node.xvf3800_power_policy_strict = False
    node._apply_xvf_power_policy(initial=True)
    assert node._xvf_policy.ok is False

    node._xvf_device = device
    assert node._read_doa_degrees() == 42.0
    assert node._xvf_last_error == 'X0D33 is 1, expected 0'   # not masked


def test_diagnostics_report_disabled_policy(real_node_with_fake_xvf):
    """Enforcement intentionally off must read as 'disabled', not as a failed verification,
    and must not raise the diagnostic level."""
    node = real_node_with_fake_xvf(FakeXvfDevice(), _ok_policy())
    node.xvf3800_power_policy_enabled = False

    node._apply_xvf_power_policy(initial=True)   # no-op when disabled
    assert node._xvf_policy is None

    kv, msg = _status_keyvalues(node)

    assert kv['xvf3800_power_policy_ok'] == 'disabled'
    assert kv['xvf3800_led_power_off'] == 'unknown'
    assert kv['xvf3800_amplifier_disabled'] == 'unknown'
    assert kv['xvf3800_mic_muted'] == 'unknown'
    assert kv['xvf3800_led_effect'] == 'unknown'
    assert msg.status[0].level == DiagnosticStatus.OK
