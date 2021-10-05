# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import uuid

from rclpy.time import Time
# TODO(@jubeira): Re-add once rosauth is ported to ROS2.
# from rosauth.srv import Authentication

import sys
import threading
import traceback
from functools import partial, wraps

from tornado import version_info as tornado_version_info
from tornado.ioloop import IOLoop
from tornado.iostream import StreamClosedError
from tornado.websocket import WebSocketHandler, WebSocketClosedError
from tornado.gen import coroutine, BadYieldError

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import json, bson

from std_msgs.msg import Int32


def _log_exception():
    """Log the most recent exception to ROS."""
    exc = traceback.format_exception(*sys.exc_info())
    RosbridgeWebSocket.node_handle.get_logger().error(''.join(exc))


def log_exceptions(f):
    """Decorator for logging exceptions to ROS."""
    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except Exception:
            _log_exception()
            raise
    return wrapper


class RosbridgeWebSocket(WebSocketHandler):
    client_id_seed = 0
    clients_connected = 0
    authenticate = False
    use_compression = False

    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = 10000000             # bytes
    unregister_timeout = 10.0               # seconds
    bson_only_mode = False
    node_handle = None
    io_loop_instance = None

    @log_exceptions
    def open(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout,
            "bson_only_mode": cls.bson_only_mode
        }
        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, cls.node_handle, parameters=parameters)
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            self.authenticated = False
            self._write_lock = threading.RLock()
            cls.client_id_seed += 1
            cls.clients_connected += 1
            self.client_id = uuid.uuid4()
            if cls.client_manager:
                cls.client_manager.add_client(self.client_id, self.request.remote_ip)
        except Exception as exc:
            cls.node_handle.get_logger().error("Unable to accept incoming connection.  Reason: {}".format(exc))

        cls.node_handle.get_logger().info("Client connected. {} clients total.".format(cls.clients_connected))
        if cls.authenticate:
            cls.node_handle.get_logger().info("Awaiting proper authentication...")

    @log_exceptions
    def on_message(self, message):
        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.on_message enter")
        self.__class__.node_handle.get_logger().info("[EVT4-1624] message={}".format(message))
        cls = self.__class__
        # check if we need to authenticate
        if cls.authenticate and not self.authenticated:
            self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.on_message authentication")
            try:
                if cls.bson_only_mode:
                    msg = bson.BSON(message).decode()
                else:
                    msg = json.loads(message)

                if msg['op'] == 'auth':
                    # check the authorization information
                    auth_srv_client = cls.node_handle.create_client(Authentication, 'authenticate')
                    auth_srv_req = Authentication.Request()
                    auth_srv_req.mac = msg['mac']
                    auth_srv_req.client = msg['client']
                    auth_srv_req.dest = msg['dest']
                    auth_srv_req.rand = msg['rand']
                    auth_srv_req.t = Time(seconds=msg['t']).to_msg()
                    auth_srv_req.level = msg['level']
                    auth_srv_req.end = Time(seconds=msg['end']).to_msg()

                    while not auth_srv_client.wait_for_service(timeout_sec=1.0):
                        cls.node_handle.get_logger().info('Authenticate service not available, waiting again...')

                    future = auth_srv_client.call_async(auth_srv_req)
                    rclpy.spin_until_future_complete(cls.node_handle, future)

                    # Log error if service could not be called.
                    if future.result() is not None:
                        self.authenticated = future.result().authenticated
                    else:
                        self.authenticated = False
                        cls.node_handle.get_logger.error('Authenticate service call failed')

                    if self.authenticated:
                        cls.node_handle.get_logger().info("Client {} has authenticated.".format(self.protocol.client_id))
                        return
                # if we are here, no valid authentication was given
                cls.node_handle.get_logger().warn(
                    "Client {} did not authenticate. Closing connection.".format(self.protocol.client_id))
                self.close()
            except:
                # proper error will be handled in the protocol class
                self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.on_message exception")
                if type(message) is bytes:
                    message = message.decode('utf-8')
                self.protocol.incoming(message)
        else:
            # no authentication required
            self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.on_message incoming")
            if type(message) is bytes:
                message = message.decode('utf-8')
            self.protocol.incoming(message)
        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.on_message exit")

    @log_exceptions
    def on_close(self):
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        if cls.client_manager:
            cls.client_manager.remove_client(self.client_id, self.request.remote_ip)
        cls.node_handle.get_logger().info("Client disconnected. {} clients total.".format(cls.clients_connected))

    def send_message(self, message):
        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.send_message enter")
        self.__class__.node_handle.get_logger().info("[EVT4-1624] message={}".format(message))

        if type(message) == bson.BSON:
            binary = True
        elif type(message) == bytearray:
            binary = True
            message = bytes(message)
        else:
            binary = False

        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.send_message lock")
        with self._write_lock:
            self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.send_message locked")
            self.io_loop_instance.add_callback(partial(self.prewrite_message, message, binary))

        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.send_message exit")

    @coroutine
    def prewrite_message(self, message, binary):
        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.prewrite_message enter")

        cls = self.__class__
        # Use a try block because the log decorator doesn't cooperate with @coroutine.
        try:

            self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.prewrite_message lock")
            with self._write_lock:
                self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.prewrite_message locked")
                future = self.write_message(message, binary)

                # When closing, self.write_message() return None even if it's an undocument output.
                # Consider it as WebSocketClosedError
                # For tornado versions <4.3.0 self.write_message() does not have a return value
                if future is None and tornado_version_info >= (4,3,0,0):
                    raise WebSocketClosedError

                yield future
        except WebSocketClosedError:
            cls.node_handle.get_logger().warn('WebSocketClosedError: Tried to write to a closed websocket',
                throttle_duration_sec=1.0)
            raise
        except StreamClosedError:
            cls.node_handle.get_logger().warn('StreamClosedError: Tried to write to a closed stream',
                throttle_duration_sec=1.0)
            raise
        except BadYieldError:
            # Tornado <4.5.0 doesn't like its own yield and raises BadYieldError.
            # This does not affect functionality, so pass silently only in this case.
            if tornado_version_info < (4, 5, 0, 0):
                pass
            else:
                _log_exception()
                raise
        except:
            _log_exception()
            raise

        self.__class__.node_handle.get_logger().info("[EVT4-1624] RosbridgeWebSocket.prewrite_message exit")

    @log_exceptions
    def check_origin(self, origin):
        return True

    @log_exceptions
    def get_compression_options(self):
        # If this method returns None (the default), compression will be disabled.
        # If it returns a dict (even an empty one), it will be enabled.
        cls = self.__class__

        if not cls.use_compression:
            return None

        return {}
