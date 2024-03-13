''' Example demonstrating receiving sensor data from a sensor server '''
import os
import json
import tempfile
import argparse
from mayfly.sensorcapture import SensorCapture

if __name__ == '__main__':
    cap = SensorCapture('config_client.json')
    while True:
        sensor_data = cap.read()
        print(sensor_data)
