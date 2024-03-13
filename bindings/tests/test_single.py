import os
import cv2
import time
import numpy as np
from mayfly.sensorcapture import SensorCapture
import turbo_colormap
import json
try:
    import torch
except:
    print('torch not available - not able to extract GPU tensors')

def main_x(configFilename):
    cap = SensorCapture(configFilename)
    while True:
        sensor_data = cap.read()
        if sensor_data.get('type','') == 'tag':
            print(sensor_data)

def main_x2(configFilename):
    cap = SensorCapture(configFilename)
    while True:
        sensor_data = cap.read()
        print(sensor_data)

def main_x3(configFilename):
    cap = SensorCapture(configFilename, stream_ids=[1,4])
    while True:
        sensor_data = cap.read()
        print(sensor_data)

def main_x4(configFilename):
    ip = '10.10.10.41'
    cfg={"id":"python","services":[{"type":"sensorClient","id":"python_client","transfers":[{"protocol":"udp","bindAddress":{"ip":"0.0.0.0","port":6000},"hostAddress":{"ip":ip,"port":8999}}]}]}
    cap = SensorCapture.from_dict(cfg)
    while True:
        sensor_data = cap.read()
        print(sensor_data)


def main_record_raw(configFilename):
    from mayfly import sensor
    ALL_STREAMS=list(range(60))
    app = sensor.AppSensor()
    queue = app.create_event_queue(ALL_STREAMS, 4)
    app.start(configFilename)

    read_index = 0
    tdata = []
    jdata = []
    while True:
        res = queue.wait(3)
        if res is None:
            continue

        print(res)
        if 'binaryData' in res:
            del res['binaryData']
        if '$data' in res:
            for tid, t in enumerate(res['$data']):
                assert isinstance(t, sensor.Tensor), 'Expected type tensor'
                #print(np.from_dlpack(t), t)
                name = 'data_%06d_%03d.bin' % (read_index, tid)
                tdata.append((name, np.from_dlpack(t)))
                #print('tensor: %s' % name)
            del res['$data']

        if 'source' in res:
            src = res['source']
            for frm in src.get('frames',[]):
                if '$data' in frm:
                    data_idx = frm['$data']['idx']
                    name = 'data_%06d_%03d.bin' % (read_index, data_idx)
                    frm['$data']['filename']=name
            jdata.append(json.dumps(res, indent=2))

        read_index+=1
        if read_index == 10:
            break

    app.stop()
    exit(1)
    for j in jdata:
        print(j)
    for name, arr in tdata:
        print(name, arr.shape, arr.dtype)


def main_xx(configFilename):
    cap = SensorCapture(configFilename)
    prev_stream_time = None
    prev_local_time = None
    frame_index_write = 0
    while True:
        t0 = time.time()
        capture_data = cap.read()
        print(capture_data)
        if capture_data is None or 'frames' not in capture_data:
            continue

        t1 = time.time()
        frame0 = capture_data['frames'][0]
        if 'image' in frame0:
            if frame0['image'].__dlpack_device__() == (2,0):
                frm = torch.from_dlpack(frame0['image']).cpu().numpy()
            else:
                frm = np.from_dlpack(frame0['image']).copy()
        elif 'depth' in frame0:
            if frame0['depth'].__dlpack_device__() == (2,0):
                frm = torch.from_dlpack(frame0['depth']).cpu().numpy()
            else:
                frm = np.from_dlpack(frame0['depth']).copy()

                TOF_MAX_DISTANCE = 4.0
                frm = turbo_colormap.interpolate(turbo_colormap.turbo_colormap_data, 1.0-frm/TOF_MAX_DISTANCE, normalize=False)
                print(frm.dtype, np.amax(frm), np.amin(frm))
                frm = frm.astype(np.float32)
                bgr = cv2.cvtColor(frm,cv2.COLOR_BGR2RGB)

                cv2.imshow('ToF', bgr)
                cv2.imwrite('Images/tof_%08d.png' % frame_index_write, (bgr*256).astype(np.uint8))
                frame_index_write += 1
                key = cv2.waitKey(1) & 0xff
                if key == 27:
                    exit(0)
                continue

        stream_id = frame0['id']
        stream_time = frame0['captureTime']
        local_time = int(1e6 * time.time())

        if prev_stream_time is None:
            prev_stream_time = stream_time

        if prev_local_time is None:
            prev_local_time = local_time

        dt_local = local_time - prev_local_time
        dt_stream = stream_time - prev_stream_time
        print('dt stream, local : ', dt_stream, dt_local, local_time - stream_time)
        print(frame0)

        prev_stream_time = stream_time
        prev_local_time = local_time

        #istr = "%6.3f : %d" % ((stream_time / 1000), frame0['frameIndex'])
        istr = "%6.3f : %d" % ((time.time() * 1000), frame0['frameIndex'])
        frm = cv2.putText(frm, istr, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (250,250,250), 2, 8)
        #cv2.imshow('video_%d'%stream_id, cv2.cvtColor(frm, cv2.COLOR_RGB2BGR))
        cv2.imshow('video', cv2.cvtColor(frm, cv2.COLOR_RGB2BGR))
        #cv2.imshow('video', frm)

        key = cv2.waitKey(1) & 0xff
        if key == 27 or key == ord('q'):
            break
        t2 = time.time()
        print("read time: %6.2fms" % (1000*(t1-t0)), "draw time: %6.2fms" % (1000*(t2-t1)))
    print('quit')
    cv2.destroyAllWindows()

def main_fb(configFilename):
    fb = open('/dev/fb0', 'wb')
    with open('/sys/class/graphics/fb0/stride') as fh:
        fbstride = int(fh.read())

    cap = SensorCapture(configFilename)

    row_start = 500
    col_start = 1500
    #_, frms = cap.read2()
    r = None

    prev_stream_time = None
    while True:
        t0 = time.time()
        capture_data = cap.read()
        if capture_data is None or 'frames' not in capture_data:
            continue

        frame0 = capture_data['frames'][0]
        if frame0['image'].__dlpack_device__() == (2,0):
            frm = torch.from_dlpack(frame0['image']).cpu().numpy()
        else:
            frm = np.from_dlpack(frame0['image']).copy()
        #frm = jpeg.decode(frm)

        if r is None:
            r = np.zeros([frm.shape[1], 4], np.uint8)

        stream_id = frame0['id']
        stream_time = frame0['captureTime']
        local_time = time.time()

        if prev_stream_time is None:
            prev_stream_time = stream_time

        print('dt stream = ', stream_time - prev_stream_time)
        prev_stream_time = stream_time

        istr = "%6.3f" % (local_time * 1000)
        #istr = "%6.3f : %d" % ((stream_time / 1000), frame0['frameIndex'])

        cv2.putText(frm, istr, (frm.shape[1]//8, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (250,250,250), 2, 8)
        t1 = time.time()
        for line, row in enumerate(frm):
            if row.shape[1] == 4:
                r[:,:3] = row[:,2::-1]
            else:
                r[:,:3] = row[:,::-1]
            fb.seek((row_start+line) * fbstride + 3*col_start, os.SEEK_SET)
            fb.write(r)
        fb.flush()
        t2 = time.time()
        print("read time: %6.2fms" % (1000*(t1-t0)), "draw time: %6.2fms" % (1000*(t2-t1)))
    fb.close()

if __name__ == '__main__':
    import sys
    filename = ''
    if len(sys.argv) > 1:
        filename=sys.argv[1]
        print('using filename: %s' % filename)

    disp = os.getenv("DISPLAY", None)
    if disp is None:
        main_fb(filename)
    else:
        main_x(filename)
