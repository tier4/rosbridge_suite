import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

topic = roslibpy.Topic(ros, "/time", "builtin_interfaces/msg/Time")
topic.subscribe(callback)

msg = {"secs": 1, "nsecs": 2}

while True:
    print("publish:", msg)
    topic.publish(roslibpy.Message(msg))
    time.sleep(1)
