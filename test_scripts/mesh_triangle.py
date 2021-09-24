import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

topic = roslibpy.Topic(ros, "/uint32_fixed_array", "shape_msgs/msg/MeshTriangle")
msg = {"vertex_indices": [1, 2, 3]}

topic.subscribe(callback)

while True:
    print("publish:", msg)
    topic.publish(roslibpy.Message(msg))
    time.sleep(1)
