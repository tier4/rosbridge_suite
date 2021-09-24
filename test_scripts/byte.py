import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

topic = roslibpy.Topic(ros, "/byte", "example_interfaces/msg/Byte")
topic.subscribe(callback)

msg = {"data": 1}

while True:
    print("publish:", msg)
    topic.publish(roslibpy.Message(msg))
    time.sleep(1)
