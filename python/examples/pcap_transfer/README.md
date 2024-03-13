## Purpose

To demonstrate that sensor data can be transferred using a network adapter in monitor mode using packet injection.
This allows for better determination of package reception timing and avoiding re-sending data which increases latency.


## The Setup

To run this example you'll need:

* A running MayFly sensor server set up for PCAP transfer (see: [Running a sensor server with PCAP](https://mayfly-ai.github.io/manual). Equipped with a wireless network adapter capable of using [monitor mode](https://en.wikipedia.org/wiki/Monitor_mode).
a configuration for the sensor server is available in the example directory: ``config_server.py``.

* A python environment with the mayfly package installed (see: [Installing the mayfly python package](https://mayfly-ai.github.io/manual) )


In the python environment run:

``python pcap_transfer.py``


You should now see a connection established and the sensor data meta-data should follow - e.g.:

```
{'type': 'video', 'id': 1, 'time': 1709208054270420, 'index': 421, 'frames': [{'type': 'video', 'id': 3, 'captureTime': 1709208054270432, 'frameIndex': 421, 'image': Tensor([480x640x3], exported: False, cuda: False)}]}
{'type': 'video', 'id': 1, 'time': 1709208054303839, 'index': 422, 'frames': [{'type': 'video', 'id': 3, 'captureTime': 1709208054303850, 'frameIndex': 422, 'image': Tensor([480x640x3], exported: False, cuda: False)}]}
{'type': 'video', 'id': 1, 'time': 1709208054337275, 'index': 423, 'frames': [{'type': 'video', 'id': 3, 'captureTime': 1709208054337286, 'frameIndex': 423, 'image': Tensor([480x640x3], exported: False, cuda: False)}]}
```
...
