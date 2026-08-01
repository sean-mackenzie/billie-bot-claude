"""Stable audio-input-device resolution by name, not ALSA card/device index (card numbers
can change between boots -- see BILLIEBOT_SENSOR_BENCH_TEST_PLAN.md UT-AUD-01).

Shared by the production audio_classifier node and the bench audio_capture_node so both
resolve the XVF3800 the same way. resolve_input_device() is pure and unit-testable without
sounddevice installed; query_input_devices() is the only impure function here.
"""

from typing import List, Optional


def resolve_input_device(devices: List[dict], name_substring: str) -> Optional[int]:
    """devices: sounddevice.query_devices()-shaped list of {'name': str, 'index': int, ...}.

    Returns the index of the first device whose name contains name_substring
    (case-insensitive). Returns None when name_substring is empty -- this preserves
    today's default behavior (None -> sounddevice/PortAudio picks the system default
    device) -- or when no device matches.
    """
    if not name_substring:
        return None
    needle = name_substring.lower()
    for device in devices:
        if needle in str(device.get('name', '')).lower():
            return device.get('index')
    return None


def query_input_devices() -> List[dict]:
    """Thin impure wrapper around sounddevice.query_devices(), kept separate from
    resolve_input_device() so the resolution logic itself is testable without
    sounddevice installed or any audio hardware attached."""
    import sounddevice as sd
    devices = sd.query_devices()
    return [dict(d, index=i) for i, d in enumerate(devices)]
