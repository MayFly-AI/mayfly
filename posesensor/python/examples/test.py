import sys
import argparse
from mayfly.posesensor import Connection
from mayfly.utils import find_serial_device

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
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

    sensor_connection = Connection()
    sensor_connection.open(dev)
    while True:
        res = sensor_connection.wait(1000)
        if res is None:
            continue
        print(res)

