#!/usr/bin/env python
import unittest
from time import sleep

import rclpy
from rclpy.node import Node
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.publishers import MultiPublisher


class TestMultiUnregistering(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        rclpy.shutdown()

    def test_publish_once(self):
        """Make sure that publishing works"""
        topic = "/test_publish_once"
        msg_type = "std_msgs/String"
        msg = {"data": "why halo thar"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        self.node.create_subscription(ros_loader.get_message_class(msg_type), topic, cb)
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(1)

        self.assertEqual(received["msg"].data, msg["data"])

    def test_publish_twice(self):
        """Make sure that publishing works"""
        topic = "/test_publish_twice"
        msg_type = "std_msgs/String"
        msg = {"data": "why halo thar"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        self.node.create_subscription(ros_loader.get_message_class(msg_type), topic, cb)
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(1)

        self.assertEqual(received["msg"].data, msg["data"])

        p.unregister()
        # TODO: Fix comment for rclpy
        # The publisher went away at time T. Here's the timeline of the events:
        # T+1 seconds - the subscriber will retry to reconnect - fail
        # T+3 seconds - the subscriber will retry to reconnect - fail
        # T+5 seconds - publish msg -> it's gone
        # T+7 seconds - the subscriber will retry to reconnect - success
        # T+8 seconds - publish msg -> OK
        # T+11 seconds - we receive the message. Looks like a bug in reconnection...
        #                https://github.com/ros/ros_comm/blob/indigo-devel/clients/rospy/src/rospy/impl/tcpros_base.py#L733
        #                That line should probably be indented.
        sleep(5)

        received["msg"] = None
        self.assertIsNone(received["msg"])
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        self.assertIsNone(received["msg"])

        sleep(3)
        p.publish(msg)
        sleep(2)
        # Next two lines should be removed when this is fixed:
        # https://github.com/ros/ros_comm/blob/indigo-devel/clients/rospy/src/rospy/impl/tcpros_base.py#L733
        self.assertIsNone(received["msg"])
        sleep(3)
        self.assertEqual(received["msg"].data, msg["data"])
