#!/usr/bin/env python
import unittest

import rclpy
from rosbridge_library.internal import pngcompression


class TestCompression(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_compress(self):
        bytes = list(range(128)) * 10000
        string = str(bytearray(bytes))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)

    def test_compress_decompress(self):
        bytes = list(range(128)) * 10000
        string = str(bytearray(bytes))
        encoded = pngcompression.encode(string)
        self.assertNotEqual(string, encoded)
        decoded = pngcompression.decode(encoded)
        self.assertEqual(string, decoded)
