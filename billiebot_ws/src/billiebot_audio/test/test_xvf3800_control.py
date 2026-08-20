"""Unit tests for the XVF3800 vendor-control module and BillieBot's power policy.

Everything here runs against fake USB devices: no ReSpeaker, no PyUSB backend, no libusb.
The only PyUSB touch point is usb.util's request-type constants inside Xvf3800Device, so a
minimal stub module is installed for the duration of the test session -- if the real PyUSB
is importable, it is used instead and the numeric expectations still hold, because the
constants are fixed by the USB spec.
"""

import sys
import types

import pytest

from billiebot_audio import xvf3800_control as xc
from billiebot_audio.xvf3800_control import (
    LEGACY_RESPEAKER_PID,
    LEGACY_RESPEAKER_VID,
    XVF3800_PID,
    XVF3800_VID,
    GpoState,
    UsbDeviceInfo,
    Xvf3800Device,
    Xvf3800Error,
    Xvf3800NotFoundError,
    ensure_billiebot_power_policy,
    open_xvf3800,
    select_xvf3800_index,
)

# USB spec values, mirrored from usb.util so the expected bmRequestType is a literal here
# rather than whatever the module under test happens to compute.
CTRL_OUT = 0x00
CTRL_IN = 0x80
CTRL_TYPE_VENDOR = 0x40
CTRL_RECIPIENT_DEVICE = 0x00

EXPECTED_WRITE_REQUEST_TYPE = CTRL_OUT | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE  # 0x40
EXPECTED_READ_REQUEST_TYPE = CTRL_IN | CTRL_TYPE_VENDOR | CTRL_RECIPIENT_DEVICE    # 0xC0


@pytest.fixture(autouse=True)
def stub_pyusb():
    """Provide usb.util if PyUSB is not installed (CI, dev laptops). Real PyUSB wins."""
    try:
        import usb.util  # noqa: F401
        yield
        return
    except ImportError:
        pass

    usb_mod = types.ModuleType('usb')
    util_mod = types.ModuleType('usb.util')
    util_mod.CTRL_OUT = CTRL_OUT
    util_mod.CTRL_IN = CTRL_IN
    util_mod.CTRL_TYPE_VENDOR = CTRL_TYPE_VENDOR
    util_mod.CTRL_RECIPIENT_DEVICE = CTRL_RECIPIENT_DEVICE
    util_mod.dispose_resources = lambda dev: None
    usb_mod.util = util_mod
    sys.modules['usb'] = usb_mod
    sys.modules['usb.util'] = util_mod
    try:
        yield
    finally:
        sys.modules.pop('usb.util', None)
        sys.modules.pop('usb', None)


class FakeUsbDevice:
    """Records every ctrl_transfer call and replays scripted read responses.

    Reads are keyed by (resource_id, command_id) so a scenario can describe the device's
    state declaratively; each key holds a list of successive responses, and the last one
    repeats once exhausted (a read-back returns the same value as long as nothing changed).
    """

    def __init__(self, responses=None):
        self.responses = {k: list(v) for k, v in (responses or {}).items()}
        self.calls = []

    def ctrl_transfer(self, bmRequestType, bRequest, wValue, wIndex, data_or_length,
                      timeout=None):
        self.calls.append((bmRequestType, bRequest, wValue, wIndex, data_or_length, timeout))
        if bmRequestType & CTRL_IN:
            command_id = wValue & ~xc.READ_COMMAND_FLAG
            queue = self.responses.get((wIndex, command_id))
            if not queue:
                raise AssertionError(
                    f'unscripted read: resource {wIndex}, command {command_id}'
                )
            return bytearray(queue.pop(0) if len(queue) > 1 else queue[0])
        return len(data_or_length)

    # --- call-log helpers ---------------------------------------------------------------

    @property
    def writes(self):
        return [c for c in self.calls if not (c[0] & CTRL_IN)]

    @property
    def reads(self):
        return [c for c in self.calls if c[0] & CTRL_IN]

    def gpo_writes(self):
        """[(pin, value), ...] for every GPO_WRITE_VALUE issued."""
        return [
            (c[4][0], c[4][1]) for c in self.writes
            if c[3] == xc.RESOURCE_GPO and c[2] == xc.CMD_GPO_WRITE_VALUE
        ]

    def led_effect_writes(self):
        return [
            c[4][0] for c in self.writes
            if c[3] == xc.RESOURCE_GPO and c[2] == xc.CMD_LED_EFFECT
        ]


def gpo_response(x0d11=0, x0d30=0, x0d31=1, x0d33=0, x0d39=0, status=0):
    """A GPO_READ_VALUES reply: status byte + five uint8 pin levels."""
    return bytes([status, x0d11, x0d30, x0d31, x0d33, x0d39])


def led_response(effect=xc.LED_EFFECT_DOA, status=0):
    return bytes([status, effect])


def doa_response(degrees, speech=1, status=0):
    return bytes([status,
                  degrees & 0xFF, (degrees >> 8) & 0xFF,
                  speech & 0xFF, (speech >> 8) & 0xFF])


def compliant_device(**gpo):
    """A device already in BillieBot's desired state: LED_EFFECT 4, X0D31 1, X0D33 0."""
    values = dict(x0d30=0, x0d31=1, x0d33=0)
    values.update(gpo)
    return FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(xc.LED_EFFECT_DOA)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [gpo_response(**values)],
    })


# ============================ device resolution ==========================================

def test_resolve_prefers_product_name_xvf3800():
    candidates = [
        UsbDeviceInfo(LEGACY_RESPEAKER_VID, LEGACY_RESPEAKER_PID, 'ReSpeaker 4 Mic Array'),
        UsbDeviceInfo(XVF3800_VID, XVF3800_PID, 'XVF3800 Voice Processor'),
    ]
    index = select_xvf3800_index(candidates, 'XVF3800')
    assert candidates[index].product_id == XVF3800_PID


def test_resolve_product_name_match_is_case_insensitive():
    candidates = [UsbDeviceInfo(0x1234, 0x5678, 'seeed xvf3800 mic array')]
    assert select_xvf3800_index(candidates, 'XVF3800') == 0


def test_resolve_falls_back_to_current_vid_pid():
    """No usable product string -- 0x2886:0x001A must still be found."""
    candidates = [
        UsbDeviceInfo(0x1234, 0x5678, 'Some Other Device'),
        UsbDeviceInfo(XVF3800_VID, XVF3800_PID, ''),
    ]
    index = select_xvf3800_index(candidates, 'XVF3800')
    assert (candidates[index].vendor_id, candidates[index].product_id) == (
        XVF3800_VID, XVF3800_PID
    )


def test_resolve_falls_back_to_legacy_vid_pid():
    candidates = [
        UsbDeviceInfo(LEGACY_RESPEAKER_VID, LEGACY_RESPEAKER_PID, 'Unknown Device'),
        UsbDeviceInfo(0x1234, 0x5678, 'Some Other Device'),
    ]
    index = select_xvf3800_index(candidates, 'XVF3800')
    assert (candidates[index].vendor_id, candidates[index].product_id) == (
        LEGACY_RESPEAKER_VID, LEGACY_RESPEAKER_PID
    )


def test_resolve_prefers_current_id_over_legacy_id():
    candidates = [
        UsbDeviceInfo(LEGACY_RESPEAKER_VID, LEGACY_RESPEAKER_PID, ''),
        UsbDeviceInfo(XVF3800_VID, XVF3800_PID, ''),
    ]
    assert select_xvf3800_index(candidates, 'XVF3800') == 1


def test_resolve_no_match_returns_none():
    assert select_xvf3800_index([UsbDeviceInfo(0x1234, 0x5678, 'Unrelated')], 'XVF3800') is None


def test_open_xvf3800_returns_the_matching_handle_not_a_look_alike():
    """Two devices with identical descriptors must not collapse: the handle returned has to
    be the one at the resolved index, which a value-equality lookup would get wrong."""
    first, second = FakeUsbDevice(), FakeUsbDevice()
    paired = [
        (first, UsbDeviceInfo(0x1234, 0x5678, '')),
        (second, UsbDeviceInfo(XVF3800_VID, XVF3800_PID, '')),
    ]
    device = open_xvf3800('XVF3800', enumerate_fn=lambda: paired)
    assert device._dev is second


def test_open_xvf3800_raises_when_absent():
    paired = [(FakeUsbDevice(), UsbDeviceInfo(0x1234, 0x5678, 'Webcam'))]
    with pytest.raises(Xvf3800NotFoundError):
        open_xvf3800('XVF3800', enumerate_fn=lambda: paired)


# ============================ transport ==================================================

def test_gpo_write_uses_exact_ctrl_transfer_arguments():
    fake = FakeUsbDevice()
    Xvf3800Device(fake).write_gpo(xc.PIN_AMPLIFIER_ENABLE, 1)

    assert len(fake.calls) == 1
    request_type, request, wvalue, windex, data, timeout = fake.calls[0]
    assert request_type == EXPECTED_WRITE_REQUEST_TYPE
    assert request == 0
    assert wvalue == xc.CMD_GPO_WRITE_VALUE      # 1
    assert windex == xc.RESOURCE_GPO             # 20
    assert list(bytearray(data)) == [31, 1]
    assert timeout == 100000


def test_gpo_read_uses_expected_arguments_and_length():
    fake = compliant_device()
    Xvf3800Device(fake).read_gpo()

    request_type, request, wvalue, windex, length, timeout = fake.calls[0]
    assert request_type == EXPECTED_READ_REQUEST_TYPE
    assert request == 0
    assert wvalue == (0x80 | xc.CMD_GPO_READ_VALUES)
    assert windex == xc.RESOURCE_GPO
    assert length == 6           # status byte + 5 uint8 pin values
    assert timeout == 100000


def test_gpo_read_values_parse_in_pin_order():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [
            gpo_response(x0d11=1, x0d30=0, x0d31=1, x0d33=0, x0d39=1)
        ],
    })
    gpo = Xvf3800Device(fake).read_gpo()

    assert (gpo.x0d11, gpo.x0d30, gpo.x0d31, gpo.x0d33, gpo.x0d39) == (1, 0, 1, 0, 1)
    assert gpo.mic_muted is False
    assert gpo.amplifier_disabled is True
    assert gpo.led_ring_powered is False


def test_gpo_state_interprets_active_levels():
    muted_and_lit = GpoState.from_values([0, 1, 0, 1, 0])
    assert muted_and_lit.mic_muted is True            # X0D30 high = muted
    assert muted_and_lit.amplifier_disabled is False  # X0D31 low = amplifier enabled
    assert muted_and_lit.led_ring_powered is True     # X0D33 high = ring powered


def test_led_effect_read_and_write():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(2)],
    })
    device = Xvf3800Device(fake)
    assert device.read_led_effect() == 2

    device.write_led_effect(xc.LED_EFFECT_DOA)
    request_type, request, wvalue, windex, data, timeout = fake.calls[-1]
    assert request_type == EXPECTED_WRITE_REQUEST_TYPE
    assert (request, wvalue, windex, timeout) == (0, xc.CMD_LED_EFFECT, xc.RESOURCE_GPO, 100000)
    assert list(bytearray(data)) == [4]


def test_doa_value_parses_little_endian_uint16_pair():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_DOA_VALUE): [doa_response(287, speech=1)],
    })
    device = Xvf3800Device(fake)
    reading = device.read_doa()

    assert reading.degrees == 287       # 0x011F -> bytes 0x1F 0x01, little-endian
    assert reading.speech_detected is True

    _, _, wvalue, windex, length, _ = fake.calls[0]
    assert wvalue == (0x80 | xc.CMD_DOA_VALUE)   # 0x92
    assert windex == xc.RESOURCE_GPO
    assert length == 5                           # status byte + 2 uint16 values


def test_read_retries_on_servicer_status_64_then_succeeds():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_DOA_VALUE): [
            doa_response(0, status=xc.STATUS_RETRY),
            doa_response(0, status=xc.STATUS_RETRY),
            doa_response(90),
        ],
    })
    slept = []
    reading = Xvf3800Device(fake, sleep=slept.append).read_doa()

    assert reading.degrees == 90
    assert len(fake.reads) == 3                  # initial + two retries
    assert slept == [xc.RETRY_SLEEP_SEC] * 2     # a real sleep between retries, no busy-spin


def test_read_gives_up_after_bounded_retries():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_DOA_VALUE): [doa_response(0, status=xc.STATUS_RETRY)],
    })
    with pytest.raises(Xvf3800Error, match='still busy'):
        Xvf3800Device(fake, sleep=lambda _s: None).read_doa()
    assert len(fake.reads) == xc.MAX_READ_ATTEMPTS


def test_read_raises_on_unknown_status():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [gpo_response(status=7)],
    })
    with pytest.raises(Xvf3800Error, match='unknown servicer status 7'):
        Xvf3800Device(fake, sleep=lambda _s: None).read_gpo()
    assert len(fake.reads) == 1                  # a non-retry status is not retried


def test_read_raises_on_short_response():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [bytes([0, 1, 0])],
    })
    with pytest.raises(Xvf3800Error, match='short response'):
        Xvf3800Device(fake).read_gpo()


# ============================ power policy ===============================================

def test_policy_performs_zero_writes_when_already_correct():
    fake = compliant_device()
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is True
    assert fake.writes == []
    assert result.writes_performed == []
    assert result.led_effect == 4
    assert result.led_power_off is True
    assert result.amplifier_disabled is True


def test_policy_writes_only_amplifier_when_x0d31_is_wrong():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(4)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [
            gpo_response(x0d31=0, x0d33=0),   # amplifier enabled (active low), ring already off
            gpo_response(x0d31=1, x0d33=0),   # after the repair
        ],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is True
    assert fake.gpo_writes() == [(31, 1)]
    assert fake.led_effect_writes() == []
    assert len(result.writes_performed) == 1
    assert 'X0D31' in result.writes_performed[0]


def test_policy_writes_only_led_power_when_x0d33_is_wrong():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(4)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [
            gpo_response(x0d31=1, x0d33=1),   # ring powered
            gpo_response(x0d31=1, x0d33=0),   # after the repair
        ],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is True
    assert fake.gpo_writes() == [(33, 0)]
    assert fake.led_effect_writes() == []
    assert result.led_power_off is True


def test_policy_restores_led_effect_to_doa_mode():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(2), led_response(4)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [gpo_response(x0d31=1, x0d33=0)],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is True
    assert fake.led_effect_writes() == [4]
    assert fake.gpo_writes() == []
    assert result.led_effect == 4


def test_policy_repairs_all_three_settings_together():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(1), led_response(4)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [
            gpo_response(x0d31=0, x0d33=1),
            gpo_response(x0d31=1, x0d33=0),
        ],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is True
    assert fake.led_effect_writes() == [4]
    assert fake.gpo_writes() == [(31, 1), (33, 0)]
    assert len(result.writes_performed) == 3


def test_policy_reads_back_to_verify_final_state():
    fake = compliant_device()
    ensure_billiebot_power_policy(Xvf3800Device(fake))

    # LED_EFFECT and GPO are each read twice: once before deciding, once to verify.
    led_reads = [c for c in fake.reads if (c[2] & ~0x80) == xc.CMD_LED_EFFECT]
    gpo_reads = [c for c in fake.reads if (c[2] & ~0x80) == xc.CMD_GPO_READ_VALUES]
    assert len(led_reads) == 2
    assert len(gpo_reads) == 2


def test_policy_reports_failure_when_readback_does_not_match():
    """The device accepts the write but does not apply it -- must not be reported ok."""
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(4)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [gpo_response(x0d31=1, x0d33=1)],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is False
    assert 'X0D33' in result.error
    assert 'LED ring still powered' in result.error
    assert fake.gpo_writes() == [(33, 0)]     # it did try


def test_policy_reports_failure_when_led_effect_readback_does_not_match():
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(0)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [gpo_response(x0d31=1, x0d33=0)],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert result.ok is False
    assert 'LED_EFFECT is 0' in result.error


def test_policy_never_raises_on_usb_error():
    class ExplodingDevice:
        def ctrl_transfer(self, *args, **kwargs):
            raise OSError('device disconnected')

    result = ensure_billiebot_power_policy(Xvf3800Device(ExplodingDevice()))
    assert result.ok is False
    assert 'device disconnected' in result.error


@pytest.mark.parametrize('x0d30', [0, 1])
@pytest.mark.parametrize('x0d31,x0d33', [(1, 0), (0, 0), (1, 1), (0, 1)])
def test_x0d30_is_never_written(x0d30, x0d31, x0d33):
    """The mute pin is observed and reported, never driven -- in any starting state."""
    fake = FakeUsbDevice({
        (xc.RESOURCE_GPO, xc.CMD_LED_EFFECT): [led_response(1), led_response(4)],
        (xc.RESOURCE_GPO, xc.CMD_GPO_READ_VALUES): [
            gpo_response(x0d30=x0d30, x0d31=x0d31, x0d33=x0d33),
            gpo_response(x0d30=x0d30, x0d31=1, x0d33=0),
        ],
    })
    result = ensure_billiebot_power_policy(Xvf3800Device(fake))

    assert all(pin != xc.PIN_MIC_MUTE for pin, _ in fake.gpo_writes())
    assert result.mic_muted is bool(x0d30)     # observed and reported separately


FORBIDDEN_OPERATIONS = (
    # Flash persistence / device reset: the policy is deliberately volatile.
    'SAVE_CONFIGURATION', 'CLEAR_CONFIGURATION', 'REBOOT',
    # USB ownership: ALSA/sounddevice must keep the audio interfaces.
    'set_configuration', 'detach_kernel_driver', 'claim_interface',
)


def test_module_defines_no_forbidden_operations():
    """There must be no way to persist this policy into XVF3800 flash, reboot the device, or
    take USB ownership away from ALSA. Checked against the parsed AST rather than raw text,
    so the module docstring can name these operations while explaining why they are absent."""
    import ast

    tree = ast.parse(open(xc.__file__).read())
    identifiers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            identifiers.add(node.id)
        elif isinstance(node, ast.Attribute):
            identifiers.add(node.attr)
        elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            identifiers.add(node.name)

    for forbidden in FORBIDDEN_OPERATIONS:
        assert forbidden not in identifiers, (
            f'xvf3800_control.py must not reference {forbidden}'
        )
        assert forbidden not in dir(xc), (
            f'xvf3800_control.py must not export {forbidden}'
        )
