## Purpose

To demonstrate processing time of flight depth data from the [ArduCam ToF Camera](https://www.arducam.com/time-of-flight-camera-raspberry-pi/).
For simplicity we're using OpenCV to visualize the depth image on screen.

## The Setup

To run this example you'll need:

* A Raspberry Pi set up with the ArduCam ToF Camera, see [Arducam repos](https://github.com/ArduCAM/Arducam_tof_camera) for instructions.
	This boils down to enabling the device tree overlay in ``/boot/config.txt``, and	connecting and powering the sensor

* A sensor server running configured with the ``config_server.json`` on the Raspberry Pi and and know its IP address

* A python environment with [OpenCV](https://pypi.org/project/opencv-python/) and the mayfly package installed, see: [Installing the mayfly python package](https://mayfly-ai.github.io/manual).

Assuming the sensor server is running on the Raspberry Pi with IP address 192.168.1.55 and the Arducam ToF Camera is attached. On your computer run:

``python tof_camera.py --ip 192.168.1.55``

You should now see a live grey-scale depth video from the ToF Camera:
