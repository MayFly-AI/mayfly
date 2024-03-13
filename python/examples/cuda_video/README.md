## Purpose

To demonstrate processing video data in CUDA device memory using pytorch without copying the decoded h264 video frame.
For simplicity we're using pytorch to calculate the average red,green,blue colors of the video input. It will be
straight forward to replace the mean value calculation with running e.g. a convolutional neural network on the video data.

## The Setup

To run this example you'll need:

* A running sensor server and and know its IP address, see: [Running a simple sensor server](https://mayfly-ai.github.io/manual).
* A python environment with the [pytorch](https://pytorch.org) and the mayfly package installed, see: [Installing the mayfly python package](https://mayfly-ai.github.io/manual).
* A GPU card with cuda capabilities.

Assuming the sensor server is running on the IP address 192.168.1.55

``python cuda_video.py --ip 192.168.1.55``


You should now see a connection established and the color averages should follow - e.g.:

```
video mean RGB: 0.31, 0.43, 0.48
video mean RGB: 0.31, 0.41, 0.47
video mean RGB: 0.30, 0.40, 0.45
video mean RGB: 0.30, 0.39, 0.44
```


## Details

The sensor client is configured to use the NVIDIA based h264 video decoder and is keeping the resulting image in CUDA memory.

``"decoder":{ "type":"nvdecode", "device":"cuda" },``

The python uses the torch API supporting [dlpack](https://github.com/dmlc/dlpack) for access to the video memory without copying:

``tensor = torch.from_dlpack(frame['image'])``


Similar API is available in tensorflow and JAX
