import numpy as np
import argparse
import cv2
from multiprocessing import Process, Queue
import math

from mayfly.posesensor import Connection
from mayfly.utils import find_serial_device
from mayfly.filters import TiltCompensatedCompass

def tilt_compensated_compass(q):
    tilt = TiltCompensatedCompass()
    size = 600
    compass_size = 250
    canvas = np.zeros([size,size,3])
    cx = size//2
    cy = size//2
    while True:
        out = q.get()
        acc = np.array([out[0],out[1],out[2]])
        mag = np.array([out[3],out[4],out[5]])

        mx, my, mz = mag
        m_len = np.sqrt(np.sum(np.square(mag)))

        ex, ey, ez, bx, by = tilt.update(acc,mag)

        # Vector components on unit circle for visualization purposes
        ez_y = -by/np.linalg.norm(np.array([bx,by]))*compass_size
        ez_x = bx/np.linalg.norm(np.array([bx,by]))*compass_size

        # In IMU frame, x axis pointing north (chips up) will show North on screen
        #ez_x_gui = -ez_y
        #ez_y_gui = ez_x

        # Antenna pointing north (chips up) will show North on screen and compass moves clockwise
        ez_x_gui = -ez_x
        ez_y_gui = -ez_y

        s = 2.*m_len/compass_size
        cv2.line(canvas, (cx,cy), (int(cx+ez_x_gui*s),int(cy-ez_y_gui*s)), (0,0,255),thickness=2)
        cv2.line(canvas, (cx,cy), (cx,cy-compass_size), (255,255,255), thickness=1)
        cv2.circle(canvas, (cx,cy), compass_size, (255,255,255), thickness=1, lineType=8)

        cv2.putText(canvas, 'N', (compass_size+40,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, 2)
        cv2.putText(canvas, 'S', (cx,size-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, 2)
        cv2.putText(canvas, 'E', (size-40,cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, 2)
        cv2.putText(canvas, 'W', (20,cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, 2)
        cv2.putText(canvas, 'mx: %d'%mx, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)
        cv2.putText(canvas, 'my: %d'%my, (10,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)
        cv2.putText(canvas, 'mz: %d'%mz, (10,90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)
        cv2.putText(canvas, '||m||: %0.2f'%m_len, (10,120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)
        cv2.putText(canvas, 'Roll (ex): %d'%(ex*(180./math.pi)), (10,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)
        cv2.putText(canvas, 'Pitch (ey): %d'%(ey*(180./math.pi)), (10,180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)
        cv2.putText(canvas, 'ez: %0.2f'%ez, (10,210), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, 2)

        cv2.imshow('Tilt compensated compass',canvas)
        cv2.waitKey(1)
        canvas[:,:,:] = 0.

# To do: Add accelerometer calibration file
if __name__ == "__main__":
    dev = find_serial_device()
    parser = argparse.ArgumentParser()
    parser.add_argument('--calib', type=str, required=True, help='You need to supply magnetometer calibration file')
    args = parser.parse_args()

    calib = np.loadtxt(args.calib)
    assert calib.shape[0] == 3
    print('Loaded calibration values:',calib)

    q = Queue(10)
    p = Process(target=tilt_compensated_compass, args=(q,))
    p.start()

    conn = Connection()
    conn.open(dev)
    while True:
        res = conn.wait(1000);
        if res is None: continue
        if res['type'] == 'imu':
            sample = np.array([res['acc'][0],
                               res['acc'][1],
                               res['acc'][2],
                               res['mag'][0]-calib[0],
                               res['mag'][1]-calib[1],
                               res['mag'][2]-calib[2]])
            q.put(np.copy(sample))
