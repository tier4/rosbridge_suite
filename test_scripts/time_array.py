import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

topic = roslibpy.Topic(ros, "/time", "rosbridge_test_msgs/msg/TestTimeArray")
topic.subscribe(callback)

msg = {"times": [{"sec": 1, "nanosec": 2}, {"sec": 3, "nanosec": 4}]}

while True:
    print("publish:", msg)
    topic.publish(roslibpy.Message(msg))
    time.sleep(1)
