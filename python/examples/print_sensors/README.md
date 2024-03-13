## Purpose

To demonstrate that sensor data can be received in python.
For now we're just printing out the values, but eventually processing the sensor data is the goal.


## The Setup

To run this example you'll need:

* A running MayFly sensor server and and know its IP address (see: [Running a simple sensor server](https://mayfly-ai.github.io/manual).
* A python environment with the mayfly package installed (see: [Installing the mayfly python package](https://mayfly-ai.github.io/manual)

Assuming the sensor server is running on the IP address 192.168.1.55:

``python print_sensors.py --ip 192.168.1.55``


You should now see a connection established and the sensor data meta-data should follow - e.g.:

```
{'type': 'video', 'id': 3, 'time': 1709208054270420, 'index': 421, 'frames': [{'type': 'video', 'id': 3, 'captureTime': 1709208054270432, 'frameIndex': 421, 'image': Tensor([480x640x3], exported: False, cuda: False)}]}
{'type': 'video', 'id': 3, 'time': 1709208054303839, 'index': 422, 'frames': [{'type': 'video', 'id': 3, 'captureTime': 1709208054303850, 'frameIndex': 422, 'image': Tensor([480x640x3], exported: False, cuda: False)}]}
{'type': 'video', 'id': 3, 'time': 1709208054337275, 'index': 423, 'frames': [{'type': 'video', 'id': 3, 'captureTime': 1709208054337286, 'frameIndex': 423, 'image': Tensor([480x640x3], exported: False, cuda: False)}]}
```
...
