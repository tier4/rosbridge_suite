import time

import roslibpy


def callback(data):
    print("subscribe:", data)


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

transient_local = roslibpy.Topic(ros, "/transient_local", "example_interfaces/msg/Int8")
transient_local.subscribe(callback)

volatile = roslibpy.Topic(ros, "/volatile", "example_interfaces/msg/Int8")
volatile.subscribe(callback)

best_effort = roslibpy.Topic(ros, "/best_effort", "example_interfaces/msg/Int8")
best_effort.subscribe(callback)

while True:
    time.sleep(1)
