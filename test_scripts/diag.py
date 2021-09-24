import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

topic = roslibpy.Topic(ros, "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray")
topic.subscribe(callback)

header = {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "map"}
status = {
    "level": 2,
    "name": "test name",
    "message": "test message",
    "hardware_id": "test id",
    "values": [{"key": "test key", "value": "test value"}],
}
msg = {"header": header, "status": [status]}

while True:
    print("publish:", msg)
    topic.publish(roslibpy.Message(msg))
    time.sleep(1)
