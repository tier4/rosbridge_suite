#!/usr/bin/env python
import time
import unittest
from json import dumps, loads

import rclpy
from rclpy.node import Node
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.capabilities.service_response import ServiceResponse
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.protocol import (
    InvalidArgumentException,
    MissingArgumentException,
    Protocol,
)
from std_srvs.srv import SetBool


class TestServiceCapabilities(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

        self.proto = Protocol(self._testMethodName)
        # change the log function so we can verify errors are logged
        self.proto.log = self.mock_log
        # change the send callback so we can access the rosbridge messages being sent
        self.proto.send = self.local_send_cb
        self.advertise = AdvertiseService(self.proto)
        self.unadvertise = UnadvertiseService(self.proto)
        self.response = ServiceResponse(self.proto)
        self.received_message = None
        self.log_entries = []

    def tearDown(self):
        rclpy.shutdown()

    def local_send_cb(self, msg):
        self.received_message = msg

    def mock_log(self, loglevel, message, _=None):
        self.log_entries.append((loglevel, message))

    def test_advertise_missing_arguments(self):
        advertise_msg = loads(dumps({"op": "advertise_service"}))
        self.assertRaises(MissingArgumentException, self.advertise.advertise_service, advertise_msg)

    def test_advertise_invalid_arguments(self):
        advertise_msg = loads(dumps({"op": "advertise_service", "type": 42, "service": None}))
        self.assertRaises(InvalidArgumentException, self.advertise.advertise_service, advertise_msg)

    def test_response_missing_arguments(self):
        response_msg = loads(dumps({"op": "service_response"}))
        self.assertRaises(MissingArgumentException, self.response.service_response, response_msg)

        # this message has the optional fields, with correct types, but not the required ones
        response_msg = loads(
            dumps({"op": "service_response", "id": "dummy_service", "values": "none"})
        )
        self.assertRaises(MissingArgumentException, self.response.service_response, response_msg)

    def test_response_invalid_arguments(self):
        response_msg = loads(dumps({"op": "service_response", "service": 5, "result": "error"}))
        self.assertRaises(InvalidArgumentException, self.response.service_response, response_msg)

    def test_advertise_service(self):
        service_path = "/set_bool_1"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.advertise.advertise_service(advertise_msg)

        # This throws an exception if the timeout is exceeded (i.e. the service is not properly advertised)
        self.node.create_client(SetBool, service_path).wait_for_service(1.0)

    def test_call_advertised_service(self):
        service_path = "/set_bool_2"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.advertise.advertise_service(advertise_msg)

        # TODO: Fix comment for rclpy
        # Call the service via rosbridge because rospy.ServiceProxy.call() is blocking
        call_service = CallService(self.proto)
        call_service.call_service(
            loads(
                dumps(
                    {
                        "op": "call_service",
                        "id": "foo",
                        "service": service_path,
                        "args": [True],
                    }
                )
            )
        )

        loop_iterations = 0
        while self.received_message is None:
            time.sleep(0.5)
            loop_iterations += 1
            if loop_iterations > 3:
                self.fail(
                    "did not receive service call rosbridge message " "after waiting 2 seconds"
                )

        self.assertFalse(self.received_message is None)
        self.assertTrue("op" in self.received_message)
        self.assertTrue(self.received_message["op"] == "call_service")
        self.assertTrue("id" in self.received_message)

        # Now send the response
        response_msg = loads(
            dumps(
                {
                    "op": "service_response",
                    "service": service_path,
                    "id": self.received_message["id"],
                    "values": {"success": True, "message": ""},
                    "result": True,
                }
            )
        )
        self.received_message = None
        self.response.service_response(response_msg)

        loop_iterations = 0
        while self.received_message is None:
            time.sleep(0.5)
            loop_iterations += 1
            if loop_iterations > 3:
                self.fail(
                    "did not receive service response rosbridge message " "after waiting 2 seconds"
                )

        self.assertFalse(self.received_message is None)
        # Rosbridge should forward the response message to the "client" (i.e. our custom send function, see setUp())
        self.assertEqual(self.received_message["op"], "service_response")
        self.assertTrue(self.received_message["result"])

    def test_unadvertise_with_live_request(self):
        service_path = "/set_bool_3"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.advertise.advertise_service(advertise_msg)

        # TODO: Fix comment for rclpy
        # Call the service via rosbridge because rospy.ServiceProxy.call() is blocking
        call_service = CallService(self.proto)
        call_service.call_service(
            loads(
                dumps(
                    {
                        "op": "call_service",
                        "id": "foo",
                        "service": service_path,
                        "args": [True],
                    }
                )
            )
        )

        loop_iterations = 0
        while self.received_message is None:
            time.sleep(0.5)
            loop_iterations += 1
            if loop_iterations > 3:
                self.fail(
                    "did not receive service call rosbridge message " "after waiting 2 seconds"
                )

        self.assertFalse(self.received_message is None)
        self.assertTrue("op" in self.received_message)
        self.assertTrue(self.received_message["op"] == "call_service")
        self.assertTrue("id" in self.received_message)

        # Now send the response
        response_msg = loads(dumps({"op": "unadvertise_service", "service": service_path}))
        self.received_message = None
        self.unadvertise.unadvertise_service(response_msg)

        loop_iterations = 0
        while self.received_message is None:
            time.sleep(0.5)
            loop_iterations += 1
            if loop_iterations > 3:
                self.fail(
                    "did not receive service response rosbridge message " "after waiting 2 seconds"
                )

        self.assertFalse(self.received_message is None)
        # Rosbridge should abort the existing service call with an error (i.e. "result" should be False)
        self.assertEqual(self.received_message["op"], "service_response")
        self.assertFalse(self.received_message["result"])
