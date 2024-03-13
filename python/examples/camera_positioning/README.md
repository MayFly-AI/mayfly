## Purpose

To demonstrate streaming and visualizing multi sensor data from: camera, Ultra-Wideband (UWB) ranging, inertial measurement unit (IMU), and magnetometer, from a MayFly Pose Camera mounted e.g. on a robot.


## The Setup

To run this example you'll need:

* A wireless Maylfy Pose Camera with a known IP address. It should be running the MayFly sensor server, enabling all necessary sensors.

* At least three additional pose sensors for enabling UWB ranging and trilateration.

* The measured distances between all pairs of the pose sensors, see: `base_measurements.json` for reference, the ids listed in the file must match the 24 bit ids of the pose sensors used.

* A python environment with [OpenCV](https://pypi.org/project/opencv-python/) for visualization, [SciPy](https://scipy.org/) for  trilateration, and the MayFly package installed, see: [Installing the mayfly python package](https://mayfly-ai.github.io/manual).


Assuming the pose camera sensor server is running on the IP address 192.168.1.55, on your development computer run the follwing command (the IP address will likely differ):

``python camera_positioning.py --ip 192.168.1.55``


You should now see two windows appear:

 * One showing the live video stream coming from the camera.

 * Another window shows the estimated 2D top-down position of the camera, based on UWB ranging and the pose sensor positions supplied in the `base_measurements.json` file.
