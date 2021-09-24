import time

import roslibpy

ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

client = roslibpy.Service(ros, "/test_service", "example_interfaces/srv/AddTwoInts")

request = roslibpy.ServiceRequest({"a": 1, "b": 2})

while True:
    print("request:", request)
    response = client.call(request, lambda response: print("response:", response))
    time.sleep(1)
