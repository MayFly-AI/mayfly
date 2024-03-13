## Purpose

To demonstrate reception of frames from two synchronized cameras.
For simplicity we're using OpenCV to visualize the image on screen.

## The Setup

To run this example you'll need:

* Two Raspberry Pi - each with a MIPI camera attached (e.g. [Raspberry Pi Global Shutter Camera] (https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/) or v2 camera.). Noting both IP adresses (e.g. 192.168.1.55 and 192.168.1.66)

* A network connection between the two Raspberry Pi.

* On your PC a python environment with [OpenCV](https://pypi.org/project/opencv-python/) and the mayfly package installed, see: [Installing the mayfly python package](https://mayfly-ai.github.io/manual).


## Running

* Designate one Raspberry Pi to be the master camera (e.g. 192.168.1.55) - and use the the `config_master_camera.json` as a template, change the `hostAddress` `ip` to point to the other Raspberry Pi IP address (e.g. 192.168.1.66). On the master camera run `mayfly` with this configuration file.

* On the other Raspberry Pi run `mayfly` using the configuration file named `config_other_camera.json`.

Assuming the master camera is running on the Raspberry Pi with IP address 192.168.1.55. On your computer run:

``python sync_cameras.py --ip 192.168.1.55``

You should now see two synchronized videos from the two raspberry pi cameras.

## Details

The master camera is running two servers: one responsible for the camera and another one for synchronizing and gathering video frames from the camera and from the other Raspberry Pi server.
The master camera clock is pushed back on the servers responsible for delivering video, they will - using a software [PLL](https://en.wikipedia.org/wiki/Phase-locked_loop) mechanism, synchronize their capture timing. Expect around 1ms jitter when the PLL has stabilized - usually within a minute.
