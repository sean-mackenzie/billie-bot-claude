from billiebot_audio.audio_device import resolve_input_device


def test_resolve_input_device_defaults_to_none_when_unset():
    devices = [{'name': 'USB Audio', 'index': 0}]
    assert resolve_input_device(devices, '') is None


def test_resolve_input_device_matches_substring():
    devices = [
        {'name': 'Built-in Microphone', 'index': 0},
        {'name': 'XVF3800 Mic Array (hw:2,0)', 'index': 1},
    ]
    assert resolve_input_device(devices, 'xvf3800') == 1


def test_resolve_input_device_no_match_returns_none():
    devices = [{'name': 'Built-in Microphone', 'index': 0}]
    assert resolve_input_device(devices, 'XVF3800') is None
