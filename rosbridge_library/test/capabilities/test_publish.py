#!/usr/bin/env python
import unittest
from json import dumps, loads
from time import sleep

import rclpy
from rclpy.node import Node
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.protocol import (
    InvalidArgumentException,
    MissingArgumentException,
    Protocol,
)
from std_msgs.msg import String


class TestAdvertise(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        rclpy.shutdown()

    def test_missing_arguments(self):
        proto = Protocol("hello")
        pub = Publish(proto)
        msg = {"op": "publish"}
        self.assertRaises(MissingArgumentException, pub.publish, msg)

        msg = {"op": "publish", "msg": {}}
        self.assertRaises(MissingArgumentException, pub.publish, msg)

    def test_invalid_arguments(self):
        proto = Protocol("hello")
        pub = Publish(proto)

        msg = {"op": "publish", "topic": 3}
        self.assertRaises(InvalidArgumentException, pub.publish, msg)

    def test_publish_works(self):
        proto = Protocol("hello")
        pub = Publish(proto)
        topic = "/test_publish_works"
        msg = {"data": "test publish works"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        rclpy.Subscriber(String, topic, cb)
        self.node.create_subscription()

        pub_msg = loads(dumps({"op": "publish", "topic": topic, "msg": msg}))
        pub.publish(pub_msg)

        sleep(0.5)
        self.assertEqual(received["msg"].data, msg["data"])
