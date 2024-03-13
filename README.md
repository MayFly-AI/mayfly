# MayFly - Low latency streaming sensor solution

Want to run advanced AI such as large vision transformers or language models on your robot or drone but you are limited
by edge compute? This repo enables you to quickly capture and send data for remote AI processing.

- Enable compute-intensive AI on robots and drones
- Quickly move sensor data from edge device to Python on remote device
- Fully open source (both software and hardware)

## Sensors currently supported

- Raspberry Pi RGB cameras on Raspberry Pi platforms
- MayFly Pose sensor (contains Bosch IMU/Magnetometer and Qorvo UWB for positioning and orientation). Two versions exist: Raspberry Pi HAT and STM32 standalone version.
- Arducam Time-of-Flight (ToF) sensor (resolution: 240x180).


## Build command on Linux
```
$ cmake . -Bbuild_reldbg
$ cmake --build build_reldbg
```

## Build command on Windows
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_GENERATOR_PLATFORM=x64 ..
$ Open mayfly.sln project in Visual Studio 17 2022 and build
```

## Install python package
```
$ pip install 'git+ssh://git@github.com/MayFly-AI/mayfly.git'
```
