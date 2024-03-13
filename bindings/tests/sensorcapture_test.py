
import os, sys, time, json
from mayfly.sensorcapture import SensorCapture

if __name__ == '__main__':
    config = ''
    if len(sys.argv) > 1:
        config=sys.argv[1]

    cap = SensorCapture(config)
    while True:
        capture = cap.read()
        print(capture)

