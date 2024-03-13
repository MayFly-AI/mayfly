import os
import time
import numpy as np
import cv2
from mayfly import video

def test():
    a = np.array([1,2,3])
    b = np.array([10,20,30])
    res = video.add(a, b)
    print(res)

    cap = video.VideoCapture()
    print(cap)

    frm = cap.read()
    print(frm.shape, frm.dtype)
    pointer, read_only_flag = frm.__array_interface__['data']
    print('%x' % (pointer), read_only_flag)
    print('%x' % frm.ctypes.data)
    cv2.imwrite('test.png', frm)
    print(frm)
    #print(frm[0,:,0])

    for idx in range(100):
        frm = cap.read()
        if idx % 10 !=0:continue
        cv2.imwrite('test_%03d.png' % idx, cv2.cvtColor(frm, cv2.COLOR_RGB2BGR))

    #frames = [ cap.read() for _ in range(500)]
    #print('number of frames', len(frames) + 1)
    #for idx,frm in enumerate(frames):
    #    cv2.imwrite('test_%03d.png' % idx, frm)
    #print(frames[2].shape)
    #del frames[2]
    #print(frames[2].shape)

def concat_frames(info, frms):
    stream_ids = [i[0] for i in info]
    sensor_times = [i[1] for i in info]
    if stream_ids[0] == stream_ids[1]:
        print("*** Same stream id in both frames ***")
    if stream_ids[0] > stream_ids[1]:
        frms = frms[::-1]
        info = info[::-1]
    return np.concatenate(list(frms), axis=1)

def run_fb():
    fb = open('/dev/fb0', 'wb')
    # /sys/class/graphics/fb0/stride
    fbstride = 16384

    cap = video.VideoCapture()

    row_start = 500
    col_start = 2500
    _, frms = cap.read2()
    r = np.zeros([frms[0].shape[1]*2, 4], np.uint8)

    while True:
        t0 = time.time()
        info, frms = cap.read2()
        captureTimes = [i[1] for i in info]
        deltaTime = max(captureTimes) - min(captureTimes)
        if(deltaTime > 1000):
            print('Bad delta time: %f'%deltaTime)
            time.sleep(0.02); # kludge

        frm = concat_frames(info, frms)
        time_str = "%6.3f" % (time.time() * 1000)
        cv2.putText(frm, time_str, (frm.shape[1]//8, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (250,250,250), 2, 8)
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

def run_x():
    cap = video.VideoCapture()
    for _ in range(10000):
        info, frms = cap.read2()
        captureTimes = [i[1] for i in info]
        deltaTime = max(captureTimes) - min(captureTimes)
        #print(info)
        #continue
        if(deltaTime > 1000):
            print('Bad delta time: %f'%deltaTime)
            time.sleep(0.1); # kludge
        if True:
            frm = concat_frames(info, frms)
            time_str = "%6.3f" % (time.time() * 1000)
            cv2.putText(frm, time_str, (frm.shape[1]//4, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (250,250,250), 2, 8)
            xoff = 0
            for t in captureTimes:
                cap_time_str = "%6.3f" % (t / 1000)
                cv2.putText(frm, cap_time_str, (frm.shape[1]//10 + xoff, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (250,250,250), 2, 8)
                xoff += 600
            cv2.imshow('frames', cv2.cvtColor(frm, cv2.COLOR_RGB2BGR))
        else:
            for inf,frm in zip(info,frms):
                stream_id, stream_time = inf
                time_str = "%6.3f" % (time.time() * 1000)
                cap_time_str = "%6.3f" % (stream_time / 1000)
                cv2.putText(frm, time_str, (frm.shape[1]//8, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (250,250,250), 2, 8)
                cv2.putText(frm, cap_time_str, (frm.shape[1]//8, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (250,250,250), 2, 8)
                cv2.imshow('video_%d'%stream_id, cv2.cvtColor(frm, cv2.COLOR_RGB2BGR))

        key = cv2.waitKey(10) & 0xff
        if key == 27 or key == ord('q'):
            break
    print('quit')
    cv2.destroyAllWindows()
    cap.stop()

if __name__ == '__main__':
    if os.getenv('DISPLAY'):
        run_x()
    else:
        run_fb()
