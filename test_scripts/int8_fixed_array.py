import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

topic = roslibpy.Topic(ros, "/fixed_array", "unique_identifier_msgs/msg/UUID")
msg = {"uuid": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]}

topic.subscribe(callback)

while True:
    print("publish:", msg)
    topic.publish(roslibpy.Message(msg))
    time.sleep(1)
