""" Connects to the posesensor via serial connection and
    uses the QuatCompass class to measure heading """
import sys
import argparse
import numpy as np

from mayfly.posesensor import Connection
from mayfly.utils import find_serial_device
from mayfly.filters import QuatCompass

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--calib', type=str, required=False,
                        help='magnetometer calibration file (3 floats)')
    parser.add_argument('--device', type=str, required=False,
                        help='specifies the serial device for the Posesensor')
    args = parser.parse_args()

    if args.device is None:
        dev = find_serial_device()
        if dev is None:
            print('Failed to find serial device for Posesensor')
            sys.exit(1)
        else:
            print("Using serial device: %s" % dev)
    else:
        dev = args.device

    if args.calib is None:
        print('Warning: the heading will be biased without calibration')
        calib = np.zeros([3], np.float32)
    else:
        calib = np.loadtxt(args.calib, np.float32)
        assert calib.shape[0] == 3
        print('Loaded calibration values:', calib)

    sensor_connection = Connection()
    sensor_connection.open(dev)
    prev_time = None

    compass = QuatCompass()

    while True:
        res = sensor_connection.wait(1000)
        if res is None:
            continue
        if res['type'] != 'imu':
            continue
        if prev_time is None:
            prev_time = res['time']
        delta_time = (res['time'] - prev_time)/1e6
        prev_time = res['time']
        acc = np.array(res['acc'])
        gyr = np.array(res['gyr'])
        mag = np.array(res['mag']) - calib
        quat_heading = compass.update(acc, gyr, mag, delta_time)
        print('Heading:', quat_heading)

    sensor_connection.close()
