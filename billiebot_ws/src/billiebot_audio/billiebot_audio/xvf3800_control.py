"""Minimal XVF3800 USB vendor-control support for BillieBot's production power policy.

BillieBot uses the ReSpeaker XVF3800 for microphone capture, YAMNet classification, and
Direction of Arrival. It does NOT use the board's WS2812 LED ring or its onboard audio
amplifier -- speaker output is a separate MAX98357A/I2S path (speaker_node.py). This module
turns "the LED ring and amplifier happen to be off at boot" into an explicit, verified
production invariant:

    LED_EFFECT = 4   (internal DoA mode, preserved)
    X0D31      = 1   (onboard audio amplifier disabled -- the enable is active low)
    X0D33      = 0   (WS2812 LED-ring power off -- the enable is active high)
    X0D30            observed and reported only, NEVER written (0 = mics live, expected)

The policy is READ-BEFORE-WRITE and VOLATILE. Nothing here saves to XVF3800 flash: there is
deliberately no SAVE_CONFIGURATION, CLEAR_CONFIGURATION, or REBOOT support, because upstream
reports undesirable device behavior after aggressive configuration experiments. Beamforming,
AEC, AGC, noise suppression, and the audio mux/routing are all left exactly as the device
booted them.

Protocol constants below were cross-checked against the current official upstream
implementation, respeaker/reSpeaker_XVF3800_USB_4MIC_ARRAY @ master
(python_control/xvf_host.py PARAMETERS table and read()/write(); host_control/README.md for
the GPO pin semantics). They agree with it exactly.

Layout follows the pure/impure split already used by audio_device.py: everything except
enumerate_usb_devices() is importable and unit-testable with no PyUSB installed and no
hardware attached.
"""

import struct
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Sequence, Tuple

# --- USB identity ----------------------------------------------------------------------
# Current Seeed XVF3800 USB 4-Mic Array.
XVF3800_VID = 0x2886
XVF3800_PID = 0x001A

# Appendix B-14 (BRINGUP_LADDER_ANALYSIS.md): this VID:PID is the older ReSpeaker
# 4-Mic/XVF3000-era array, kept only as a compatibility fallback -- see
# select_xvf3800_index(), which prefers the product string and then the current ID.
LEGACY_RESPEAKER_VID = 0x2886
LEGACY_RESPEAKER_PID = 0x0018

DEFAULT_PRODUCT_SUBSTRING = 'XVF3800'

# --- Vendor control transport ----------------------------------------------------------
CTRL_REQUEST = 0            # bRequest is always 0 for this device
CTRL_TIMEOUT_MS = 100000
READ_COMMAND_FLAG = 0x80    # wValue = 0x80 | command_id marks a read

STATUS_SUCCESS = 0
STATUS_RETRY = 64
MAX_READ_ATTEMPTS = 100     # matches the official host utility's bound
RETRY_SLEEP_SEC = 0.01

# --- Resource / command IDs (upstream PARAMETERS table) --------------------------------
RESOURCE_GPO = 20

CMD_GPO_READ_VALUES = 0     # ro, uint8 x5
CMD_GPO_WRITE_VALUE = 1     # wo, uint8 x2 -> [pin_number, value]
CMD_LED_EFFECT = 12         # rw, uint8 x1
CMD_DOA_VALUE = 18          # ro, uint16 x2 -> [doa_degrees, speech_detected]

SIZEOF_UINT8 = 1
SIZEOF_UINT16 = 2

# GPO_READ_VALUES returns the five pins in this fixed order.
GPO_PIN_ORDER = (11, 30, 31, 33, 39)

PIN_MIC_MUTE = 30           # high = microphones muted + red mute LED on
PIN_AMPLIFIER_ENABLE = 31   # active low: 0 = amplifier enabled, 1 = disabled
PIN_LED_RING_POWER = 33     # active high: 1 = WS2812 ring powered, 0 = unpowered

LED_EFFECT_DOA = 4          # 0=off 1=breath 2=rainbow 3=single color 4=doa

# --- BillieBot production invariant -----------------------------------------------------
DESIRED_LED_EFFECT = LED_EFFECT_DOA
DESIRED_AMPLIFIER_ENABLE = 1    # amplifier disabled
DESIRED_LED_RING_POWER = 0      # LED ring unpowered

DOA_MIN_DEG = 0
DOA_MAX_DEG = 359


class Xvf3800Error(RuntimeError):
    """Any XVF3800 vendor-control failure: bad status, short read, retry exhaustion."""


class Xvf3800NotFoundError(Xvf3800Error):
    """No XVF3800 (or legacy ReSpeaker) matched among the enumerated USB devices."""


@dataclass
class UsbDeviceInfo:
    vendor_id: int
    product_id: int
    product_name: str = ''


@dataclass
class GpoState:
    """The five GPO pin levels, in GPO_READ_VALUES order."""

    x0d11: int
    x0d30: int
    x0d31: int
    x0d33: int
    x0d39: int

    @classmethod
    def from_values(cls, values: Sequence[int]) -> 'GpoState':
        if len(values) != len(GPO_PIN_ORDER):
            raise Xvf3800Error(
                f'GPO_READ_VALUES returned {len(values)} values, expected '
                f'{len(GPO_PIN_ORDER)}'
            )
        return cls(*(int(v) for v in values))

    @property
    def mic_muted(self) -> bool:
        """X0D30 high means the mute circuit is engaged and the red mute LED is lit."""
        return bool(self.x0d30)

    @property
    def amplifier_disabled(self) -> bool:
        """X0D31 is an active-low enable, so 1 means the onboard amplifier is off."""
        return self.x0d31 == DESIRED_AMPLIFIER_ENABLE

    @property
    def led_ring_powered(self) -> bool:
        """X0D33 is an active-high power control for the WS2812 ring."""
        return bool(self.x0d33)


@dataclass
class DoaReading:
    degrees: int
    speech_detected: bool


@dataclass
class PowerPolicyResult:
    """Structured outcome of ensure_billiebot_power_policy(), for logging + diagnostics."""

    ok: bool = False
    led_effect: Optional[int] = None
    gpo: Optional[GpoState] = None
    writes_performed: List[str] = field(default_factory=list)
    error: str = ''
    device: Optional[UsbDeviceInfo] = None

    @property
    def led_power_off(self) -> Optional[bool]:
        return None if self.gpo is None else not self.gpo.led_ring_powered

    @property
    def amplifier_disabled(self) -> Optional[bool]:
        return None if self.gpo is None else self.gpo.amplifier_disabled

    @property
    def mic_muted(self) -> Optional[bool]:
        return None if self.gpo is None else self.gpo.mic_muted


def select_xvf3800_index(candidates: List[UsbDeviceInfo],
                         product_substring: str) -> Optional[int]:
    """Pure resolver. Returns the *index* of the chosen candidate, three tiers deep:

    1. a device whose product name contains product_substring (case-insensitive), e.g.
       'XVF3800' -- the most specific and least brittle match;
    2. the current XVF3800 VID:PID 0x2886:0x001A;
    3. the legacy 0x2886:0x0018 VID:PID (Appendix B-14), so hardware that happened to
       enumerate under the old ID never regresses.

    An index rather than the UsbDeviceInfo itself, because callers map back to a live PyUSB
    handle: two arrays sharing a VID/PID/product string (or two whose descriptor read failed
    and both yield '') compare equal as dataclasses, and a value-based lookup would silently
    return the wrong handle.
    """
    needle = product_substring.lower() if product_substring else ''
    if needle:
        for i, c in enumerate(candidates):
            if needle in c.product_name.lower():
                return i
    for vid, pid in ((XVF3800_VID, XVF3800_PID),
                     (LEGACY_RESPEAKER_VID, LEGACY_RESPEAKER_PID)):
        for i, c in enumerate(candidates):
            if c.vendor_id == vid and c.product_id == pid:
                return i
    return None


def enumerate_usb_devices() -> List[Tuple[Any, UsbDeviceInfo]]:
    """Thin impure wrapper around usb.core.find(find_all=True), kept separate from
    select_xvf3800_index() so the resolution logic is testable without PyUSB installed or
    any hardware attached. Pairs each live handle with its descriptor info so callers never
    have to look a handle back up by value.

    A device whose product-string descriptor cannot be read (permissions, a device that
    does not expose one) is still returned, with an empty product_name -- it may still match
    on VID/PID.
    """
    import usb.core
    import usb.util

    paired = []
    for dev in usb.core.find(find_all=True):
        product_name = ''
        try:
            product_name = usb.util.get_string(dev, dev.iProduct) or ''
        except Exception:
            pass
        paired.append((dev, UsbDeviceInfo(dev.idVendor, dev.idProduct, product_name)))
    return paired


class Xvf3800Device:
    """Vendor control-transfer wrapper around one PyUSB device handle.

    Deliberately does NOT call set_configuration(), detach a kernel driver, or claim any
    interface: endpoint-0 vendor control transfers need none of that, and ALSA/sounddevice
    must retain ownership of the device's USB audio interfaces or microphone capture breaks.
    """

    def __init__(self, dev: Any, info: Optional[UsbDeviceInfo] = None,
                 sleep: Callable[[float], None] = time.sleep):
        self._dev = dev
        self.info = info
        self._sleep = sleep

    # --- low-level transport ------------------------------------------------------------

    def _read(self, command_id: int, resource_id: int, count: int,
              item_size: int) -> Tuple[int, ...]:
        """Vendor IN transfer with a bounded servicer-status retry loop.

        The response is [status_byte][payload]; status 0 is success and status 64 means the
        servicer wants the host to ask again. Retries are bounded and spaced by a short
        sleep rather than busy-spinning, matching the official host utility.
        """
        import usb.util

        length = count * item_size + 1  # +1 for the leading servicer status byte
        request_type = (usb.util.CTRL_IN
                        | usb.util.CTRL_TYPE_VENDOR
                        | usb.util.CTRL_RECIPIENT_DEVICE)
        wvalue = READ_COMMAND_FLAG | command_id

        attempts = 1
        response = self._dev.ctrl_transfer(
            request_type, CTRL_REQUEST, wvalue, resource_id, length, CTRL_TIMEOUT_MS
        )
        while True:
            raw = bytes(response)
            if not raw:
                raise Xvf3800Error(
                    f'empty response reading command {command_id} on resource {resource_id}'
                )
            status = raw[0]
            if status == STATUS_SUCCESS:
                break
            if status != STATUS_RETRY:
                raise Xvf3800Error(
                    f'unknown servicer status {status} reading command {command_id} on '
                    f'resource {resource_id}'
                )
            if attempts >= MAX_READ_ATTEMPTS:
                raise Xvf3800Error(
                    f'servicer still busy after {MAX_READ_ATTEMPTS} attempts reading '
                    f'command {command_id} on resource {resource_id}'
                )
            attempts += 1
            self._sleep(RETRY_SLEEP_SEC)
            response = self._dev.ctrl_transfer(
                request_type, CTRL_REQUEST, wvalue, resource_id, length, CTRL_TIMEOUT_MS
            )

        payload = raw[1:]
        if len(payload) < count * item_size:
            raise Xvf3800Error(
                f'short response reading command {command_id} on resource {resource_id}: '
                f'got {len(payload)} payload bytes, expected {count * item_size}'
            )
        fmt = '<' + ('B' if item_size == SIZEOF_UINT8 else 'H') * count
        return struct.unpack(fmt, payload[:count * item_size])

    def _write(self, command_id: int, resource_id: int, payload: Sequence[int]) -> None:
        import usb.util

        request_type = (usb.util.CTRL_OUT
                        | usb.util.CTRL_TYPE_VENDOR
                        | usb.util.CTRL_RECIPIENT_DEVICE)
        self._dev.ctrl_transfer(
            request_type, CTRL_REQUEST, command_id, resource_id,
            bytes(bytearray(int(v) & 0xFF for v in payload)), CTRL_TIMEOUT_MS
        )

    # --- typed accessors ----------------------------------------------------------------

    def read_gpo(self) -> GpoState:
        values = self._read(CMD_GPO_READ_VALUES, RESOURCE_GPO,
                            len(GPO_PIN_ORDER), SIZEOF_UINT8)
        return GpoState.from_values(values)

    def write_gpo(self, pin: int, value: int) -> None:
        """Set one GPO pin. X0D30 (the mic-mute circuit) is never written by this module's
        policy -- muting the microphones would defeat the whole audio pipeline -- but the
        primitive itself stays general."""
        self._write(CMD_GPO_WRITE_VALUE, RESOURCE_GPO, [pin, value])

    def read_led_effect(self) -> int:
        return int(self._read(CMD_LED_EFFECT, RESOURCE_GPO, 1, SIZEOF_UINT8)[0])

    def write_led_effect(self, effect: int) -> None:
        self._write(CMD_LED_EFFECT, RESOURCE_GPO, [effect])

    def read_doa(self) -> DoaReading:
        degrees, speech = self._read(CMD_DOA_VALUE, RESOURCE_GPO, 2, SIZEOF_UINT16)
        return DoaReading(degrees=int(degrees), speech_detected=bool(speech))

    def close(self) -> None:
        """Release the PyUSB handle. Safe to call more than once, and safe when PyUSB is
        unavailable -- callers use this on error paths where re-resolution must not be
        blocked by a secondary failure."""
        try:
            import usb.util
            usb.util.dispose_resources(self._dev)
        except Exception:
            pass


def open_xvf3800(product_substring: str = DEFAULT_PRODUCT_SUBSTRING,
                 enumerate_fn: Callable[[], List[Tuple[Any, UsbDeviceInfo]]] = None,
                 sleep: Callable[[float], None] = time.sleep) -> Xvf3800Device:
    """Discover the XVF3800 and wrap it. Raises Xvf3800NotFoundError if nothing matches."""
    enumerate_fn = enumerate_fn or enumerate_usb_devices
    paired = list(enumerate_fn())
    index = select_xvf3800_index([info for _, info in paired], product_substring)
    if index is None:
        raise Xvf3800NotFoundError(
            f'no XVF3800 found among {len(paired)} USB devices (product substring '
            f"'{product_substring}', VID:PID {XVF3800_VID:#06x}:{XVF3800_PID:#06x}, "
            f'legacy {LEGACY_RESPEAKER_VID:#06x}:{LEGACY_RESPEAKER_PID:#06x})'
        )
    dev, info = paired[index]
    return Xvf3800Device(dev, info, sleep=sleep)


def ensure_billiebot_power_policy(device: Xvf3800Device) -> PowerPolicyResult:
    """Apply and verify BillieBot's conservative XVF3800 power policy. Never raises.

    Read-before-write: the device is read first and a write is issued only for a setting
    that is actually wrong, so a compliant device sees ZERO writes. Everything is volatile --
    no SAVE_CONFIGURATION, so a power cycle simply returns the device to its own defaults and
    the next startup re-applies the policy.

    X0D30 (microphone mute) is read and reported but never written: the expected normal
    state is 0, and automatically driving it would risk silencing the array.
    """
    result = PowerPolicyResult(device=device.info)
    try:
        led_effect = device.read_led_effect()
        gpo = device.read_gpo()

        if led_effect != DESIRED_LED_EFFECT:
            device.write_led_effect(DESIRED_LED_EFFECT)
            result.writes_performed.append(
                f'LED_EFFECT {led_effect} -> {DESIRED_LED_EFFECT}'
            )

        if gpo.x0d31 != DESIRED_AMPLIFIER_ENABLE:
            device.write_gpo(PIN_AMPLIFIER_ENABLE, DESIRED_AMPLIFIER_ENABLE)
            result.writes_performed.append(
                f'X0D31 {gpo.x0d31} -> {DESIRED_AMPLIFIER_ENABLE} (amplifier disabled)'
            )

        if gpo.x0d33 != DESIRED_LED_RING_POWER:
            device.write_gpo(PIN_LED_RING_POWER, DESIRED_LED_RING_POWER)
            result.writes_performed.append(
                f'X0D33 {gpo.x0d33} -> {DESIRED_LED_RING_POWER} (LED ring power off)'
            )

        # Read back unconditionally: the verification is the point, and a device that
        # accepted a write without applying it must not be reported as compliant.
        result.led_effect = device.read_led_effect()
        result.gpo = device.read_gpo()

        mismatches = []
        if result.led_effect != DESIRED_LED_EFFECT:
            mismatches.append(
                f'LED_EFFECT is {result.led_effect}, expected {DESIRED_LED_EFFECT}'
            )
        if result.gpo.x0d31 != DESIRED_AMPLIFIER_ENABLE:
            mismatches.append(
                f'X0D31 is {result.gpo.x0d31}, expected {DESIRED_AMPLIFIER_ENABLE} '
                '(amplifier not disabled)'
            )
        if result.gpo.x0d33 != DESIRED_LED_RING_POWER:
            mismatches.append(
                f'X0D33 is {result.gpo.x0d33}, expected {DESIRED_LED_RING_POWER} '
                '(LED ring still powered)'
            )

        if mismatches:
            result.ok = False
            result.error = 'XVF3800 power policy verification failed: ' + '; '.join(mismatches)
        else:
            result.ok = True
    except Exception as e:
        result.ok = False
        result.error = f'{type(e).__name__}: {e}'
    return result


def describe_policy_result(result: PowerPolicyResult) -> str:
    """One-line human summary for node logs."""
    if not result.ok:
        return f'XVF3800 power policy NOT satisfied -- {result.error or "unknown error"}'
    writes = ', '.join(result.writes_performed) if result.writes_performed else 'no writes needed'
    mic = 'MUTED' if result.mic_muted else 'live'
    return (
        f'XVF3800 power policy verified (LED_EFFECT={result.led_effect}, '
        f'X0D31=1 amplifier disabled, X0D33=0 LED ring off; {writes}; '
        f'X0D30 observed: microphones {mic})'
    )
