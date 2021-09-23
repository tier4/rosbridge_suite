#!/usr/bin/env python
import time
import unittest
from json import dumps, loads

import rclpy
from rcl_interfaces.srv import GetParameterTypes
from rclpy.clock import Clock, Duration
from rclpy.node import Node
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.protocol import (
    InvalidArgumentException,
    MissingArgumentException,
    Protocol,
)
from std_srvs.srv import SetBool


class TestCallService(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        rclpy.shutdown()

    def test_missing_arguments(self):
        proto = Protocol("test_missing_arguments")
        s = CallService(proto)
        msg = loads(dumps({"op": "call_service"}))
        self.assertRaises(MissingArgumentException, s.call_service, msg)

    def test_invalid_arguments(self):
        proto = Protocol("test_invalid_arguments")
        s = CallService(proto)

        msg = loads(dumps({"op": "call_service", "service": 3}))
        self.assertRaises(InvalidArgumentException, s.call_service, msg)

    def test_call_service_works(self):
        # Prepare to call the service the 'proper' way
        p = self.node.create_client(
            GetParameterTypes, self.node.get_name() + "/get_parameter_types"
        )
        p.wait_for_service()
        time.sleep(1.0)

        proto = Protocol("test_call_service_works")
        s = CallService(proto)
        msg = loads(
            dumps({"op": "call_service", "service": self.node.get_name() + "/get_parameter_types"})
        )

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None):
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(msg)

        timeout = 5.0
        start = Clock().now()
        while Clock().now() - start < Duration(seconds=timeout):
            if received["arrived"]:
                break
            time.sleep(0.1)

        # The rosbridge service call actually causes another logger to appear,
        # so do the "regular" service call after that.
        ret = p()

        self.assertTrue(received["msg"]["result"])
        for x, y in zip(ret.loggers, received["msg"]["values"]["loggers"]):
            self.assertEqual(x.name, y["name"])
            self.assertEqual(x.level, y["level"])

    def test_call_service_fail(self):
        # Dummy service that instantly fails
        service_server = self.node.create_service(SetBool, "set_bool_fail", lambda req: None)

        proto = Protocol("test_call_service_fail")
        s = CallService(proto)
        send_msg = loads(
            dumps(
                {
                    "op": "call_service",
                    "service": self.node.get_name() + "/set_bool_fail",
                    "args": "[ true ]",
                }
            )
        )

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None):
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(send_msg)

        timeout = 5.0
        start = Clock().now()
        while Clock().now() - start < Duration(seconds=timeout):
            if received["arrived"]:
                break
            time.sleep(0.1)

        self.assertFalse(received["msg"]["result"])
