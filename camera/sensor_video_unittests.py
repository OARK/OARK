#!/usr/bin/env python

# Basic unit tests for the camera streaming.
import unittest

from sensor_video import _test_device_available

class TestVideoSensor(unittest.TestCase):

    # Test we get true on something that's always readable.
    def test_device_available(self):
        self.assertTrue(_test_device_available('sensor_video_unittests.py'))

    def test_not_file_false(self):
        self.assertFalse(_test_device_available('/dev/null'))

if __name__ == '__main__':
    unittest.main()
