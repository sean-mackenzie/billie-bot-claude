import random
from unittest.mock import MagicMock

import pytest
import rclpy

from billiebot_audio.audio_classifier import (
    LEGACY_RESPEAKER_PID,
    LEGACY_RESPEAKER_VID,
    AudioClassifier,
    UsbDeviceInfo,
    resolve_doa_device,
)


def test_resolve_doa_device_prefers_xvf3800():
    candidates = [
        UsbDeviceInfo(vendor_id=LEGACY_RESPEAKER_VID, product_id=LEGACY_RESPEAKER_PID,
                      product_name='ReSpeaker 4 Mic Array'),
        UsbDeviceInfo(vendor_id=0x2886, product_id=0x001A, product_name='XVF3800 Voice Processor'),
    ]
    chosen = resolve_doa_device(candidates, 'XVF3800')
    assert chosen.product_id == 0x001A


def test_resolve_doa_device_falls_back_to_legacy_id():
    candidates = [
        UsbDeviceInfo(vendor_id=LEGACY_RESPEAKER_VID, product_id=LEGACY_RESPEAKER_PID,
                      product_name='Unknown Device'),
        UsbDeviceInfo(vendor_id=0x1234, product_id=0x5678, product_name='Some Other Device'),
    ]
    chosen = resolve_doa_device(candidates, 'XVF3800')
    assert chosen.vendor_id == LEGACY_RESPEAKER_VID
    assert chosen.product_id == LEGACY_RESPEAKER_PID


def test_resolve_doa_device_no_match_returns_none():
    candidates = [UsbDeviceInfo(vendor_id=0x1234, product_id=0x5678, product_name='Unrelated')]
    assert resolve_doa_device(candidates, 'XVF3800') is None


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
