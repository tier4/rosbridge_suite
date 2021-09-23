#!/usr/bin/env python
import unittest
from json import dumps, loads
from time import sleep

import rclpy
from ros2topic.api import get_topic_names_and_types
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.publishers import manager
from rosbridge_library.protocol import (
    InvalidArgumentException,
    MissingArgumentException,
    Protocol,
)


class TestAdvertise(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        manager.unregister_timeout = 1.0

    def tearDown(self):
        rclpy.shutdown()

    def is_topic_published(self, topicname):
        return topicname in dict(get_topic_names_and_types()).keys()

    def test_missing_arguments(self):
        proto = Protocol("hello")
        adv = Advertise(proto)
        msg = {"op": "advertise"}
        self.assertRaises(MissingArgumentException, adv.advertise, loads(dumps(msg)))

        msg = {"op": "advertise", "topic": "/jon"}
        self.assertRaises(MissingArgumentException, adv.advertise, loads(dumps(msg)))

        msg = {"op": "advertise", "type": "std_msgs/String"}
        self.assertRaises(MissingArgumentException, adv.advertise, loads(dumps(msg)))

    def test_invalid_arguments(self):
        proto = Protocol("hello")
        adv = Advertise(proto)

        msg = {"op": "advertise", "topic": 3, "type": "std_msgs/String"}
        self.assertRaises(InvalidArgumentException, adv.advertise, loads(dumps(msg)))

        msg = {"op": "advertise", "topic": "/jon", "type": 3}
        self.assertRaises(InvalidArgumentException, adv.advertise, loads(dumps(msg)))

    def test_invalid_msg_typestrings(self):
        invalid = [
            "",
            "/",
            "//",
            "///",
            "////",
            "/////",
            "bad",
            "stillbad",
            "not/better/still",
            "not//better//still",
            "not///better///still",
            "better/",
            "better//",
            "better///",
            "/better",
            "//better",
            "///better",
            r"this\isbad",
            "\\",
        ]

        proto = Protocol("hello")
        adv = Advertise(proto)

        for invalid_type in invalid:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_typestrings",
                "type": invalid_type,
            }
            self.assertRaises(
                ros_loader.InvalidTypeStringException, adv.advertise, loads(dumps(msg))
            )

    def test_invalid_msg_package(self):
        nonexistent = [
            "wangle_msgs/Jam",
            "whistleblower_msgs/Document",
            "sexual_harrassment_msgs/UnwantedAdvance",
            "coercion_msgs/Bribe",
            "airconditioning_msgs/Cold",
            "pr2thoughts_msgs/Escape",
        ]

        proto = Protocol("hello")
        adv = Advertise(proto)

        for invalid_type in nonexistent:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_package",
                "type": invalid_type,
            }
            self.assertRaises(ros_loader.InvalidPackageException, adv.advertise, loads(dumps(msg)))

    def test_invalid_msg_module(self):
        no_msgs = [
            "roslib/Time",
            "roslib/Duration",
            "roslib/Header",
            "std_srvs/ConflictedMsg",
            "topic_tools/MessageMessage",
        ]

        proto = Protocol("hello")
        adv = Advertise(proto)

        for invalid_type in no_msgs:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_module",
                "type": invalid_type,
            }
            self.assertRaises(ros_loader.InvalidModuleException, adv.advertise, loads(dumps(msg)))

    def test_invalid_msg_classes(self):
        nonexistent = [
            "rclcpp/Time",
            "rclcpp/Duration",
            "rclcpp/Header",
            "rclpy/Time",
            "rclpy/Duration",
            "rclpy/Header",
            "std_msgs/Spool",
            "geometry_msgs/Tetrahedron",
            "sensor_msgs/TelepathyUnit",
        ]

        proto = Protocol("hello")
        adv = Advertise(proto)

        for invalid_type in nonexistent:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_classes",
                "type": invalid_type,
            }
            self.assertRaises(ros_loader.InvalidClassException, adv.advertise, loads(dumps(msg)))

    def test_valid_msg_classes(self):
        assortedmsgs = [
            "geometry_msgs/Pose",
            "actionlib_msgs/GoalStatus",
            "geometry_msgs/WrenchStamped",
            "stereo_msgs/DisparityImage",
            "nav_msgs/OccupancyGrid",
            "geometry_msgs/Point32",
            "std_msgs/String",
            "trajectory_msgs/JointTrajectoryPoint",
            "diagnostic_msgs/KeyValue",
            "visualization_msgs/InteractiveMarkerUpdate",
            "nav_msgs/GridCells",
            "sensor_msgs/PointCloud2",
        ]

        proto = Protocol("hello")
        adv = Advertise(proto)

        for valid_type in assortedmsgs:
            msg = {"op": "advertise", "topic": "/" + valid_type, "type": valid_type}
            adv.advertise(loads(dumps(msg)))
            adv.unadvertise(loads(dumps(msg)))

    def test_do_advertise(self):
        proto = Protocol("hello")
        adv = Advertise(proto)
        topic = "/test_do_advertise"
        type = "std_msgs/String"

        msg = {"op": "advertise", "topic": topic, "type": type}
        adv.advertise(loads(dumps(msg)))
        self.assertTrue(self.is_topic_published(topic))
        adv.unadvertise(loads(dumps(msg)))
        self.assertTrue(self.is_topic_published(topic))
        sleep(manager.unregister_timeout * 1.1)
        self.assertFalse(self.is_topic_published(topic))
