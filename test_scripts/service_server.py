import time

import roslibpy


def handler(request, response):
    print("service called:", request)
    response["sum"] = request["a"] + request["b"]
    return True


ros = roslibpy.Ros(host="localhost", port=9090)
ros.run()

service = roslibpy.Service(ros, "/test_service", "example_interfaces/srv/AddTwoInts")
service.advertise(handler)

while True:
    print("loop")
    time.sleep(1)
