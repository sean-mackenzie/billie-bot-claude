"""Test that all BillieBot interfaces can be imported."""

import unittest


class TestInterfaces(unittest.TestCase):
    def test_import_messages(self):
        from billiebot_interfaces.msg import DogDetection3D
        from billiebot_interfaces.msg import DogState
        from billiebot_interfaces.msg import AudioEvent
        from billiebot_interfaces.msg import ThermalBlob
        from billiebot_interfaces.msg import BatteryStatus
        from billiebot_interfaces.msg import MissionStatus

    def test_import_services(self):
        from billiebot_interfaces.srv import EStop
        from billiebot_interfaces.srv import SetMode
        from billiebot_interfaces.srv import GetDogState

    def test_import_actions(self):
        from billiebot_interfaces.action import ApproachDog
        from billiebot_interfaces.action import Retreat
        from billiebot_interfaces.action import Speak
        from billiebot_interfaces.action import DispenseTreat
        from billiebot_interfaces.action import PatrolWaypoints

    def test_dog_state_constants(self):
        from billiebot_interfaces.msg import DogState
        self.assertEqual(DogState.NOT_FOUND, 0)
        self.assertEqual(DogState.SLEEPING, 1)
        self.assertEqual(DogState.RESTING, 2)
        self.assertEqual(DogState.ACTIVE, 3)
        self.assertEqual(DogState.BARKING, 4)
        self.assertEqual(DogState.EATING, 5)

    def test_audio_event_constants(self):
        from billiebot_interfaces.msg import AudioEvent
        self.assertEqual(AudioEvent.BARK, 0)
        self.assertEqual(AudioEvent.WHINE, 1)
        self.assertEqual(AudioEvent.HOWL, 2)
        self.assertEqual(AudioEvent.LOUD_NOISE, 3)
        self.assertEqual(AudioEvent.SILENCE, 4)

    def test_mission_mode_constants(self):
        from billiebot_interfaces.msg import MissionStatus
        self.assertEqual(MissionStatus.IDLE, 0)
        self.assertEqual(MissionStatus.PATROL, 1)
        self.assertEqual(MissionStatus.SAFE, 5)


if __name__ == '__main__':
    unittest.main()
