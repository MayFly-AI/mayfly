import os
import json
import time
import numpy as np
import cv2
from mayfly import video
import torch
#import jax
#import jax.dlpack

graph = np.zeros([250,2], dtype=float)

def draw_graph(img_, idx, val1, val2, scale):
    global graph
    voff, width = 100, 120
    mlen = len(graph)
    midx = idx%mlen
    graph[midx,0] = val1
    graph[midx,1] = val2
    img_[voff:voff+mlen, 0:width, :2] = 0
    img_[voff+midx, :width, 2] = 255
    for idx, x in enumerate(graph):
        w1 = int(np.clip(x[0]*scale, 0, width))
        img_[voff+idx, 0:w1, 0] = 210
        w2 = int(np.clip(x[1]*scale, 0, width))
        img_[voff+idx, width-w2:width, 1] = 210

def run_x(cfg):
    cap = video.VideoCapture(cfg)
    prev_stream_time = 0
    prev_wall_time = 0
    sys_str = ''
    idx = 0;

    while True:
        frm_info, frm = cap.read_single_cuda()
        #frm_info, frm = cap.read_single_ram()

        string_info = cap.info()
        if string_info:
            print('string_info = ',string_info)
            info = json.loads(string_info)
            if 'System' in info:
                sys_str = ', '.join(['%s: %d' % (k,v) for k,v in info['System'].items()])
            else:
                print('unknown info: %s' % str(info))

        if frm_info is None:
            idx+=1
            continue

        stream_id, stream_time_us = frm_info
        wall_time_ms = 1000.0*time.time()
        dt_wall_ms = wall_time_ms - prev_wall_time
        prev_wall_time = wall_time_ms

        dt_sensor_ms = stream_time_us/1000.0-prev_stream_time
        prev_stream_time = stream_time_us/1000.0

        # np
        #arr = np.from_dlpack(frm).copy()
        # torch
        arr = torch.from_dlpack(frm).cpu().numpy()
        # jax
        #tens = jax.dlpack.from_dlpack(frm.__dlpack__())
        #tens = jax.device_put(tens, device=jax.devices("cpu")[0])
        #arr = np.array(tens)

        img = arr[:,:,:3].copy()
        h, w, _ = img.shape
        cap_dt_str = '%dx%d cap.dt: %6.3fms, dis.dt: %6.3fms' % (w,h, dt_sensor_ms, dt_wall_ms)
        img = cv2.putText(img, cap_dt_str, (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (250,250,250), 2, 8)
        img = cv2.putText(img, '%12.6f' % (stream_time_us/1000), (25, 255), cv2.FONT_HERSHEY_SIMPLEX, 4, (250,250,250), 4, 8)
        if sys_str:
            img = cv2.putText(img, sys_str, (25, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (250,250,250), 2, 8)
        draw_graph(img, idx, dt_sensor_ms, dt_wall_ms, 1)

        win_name = 'video_%d'%stream_id
        cv2.imshow(win_name, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        key = cv2.waitKey(10) & 0xff
        if key == 27 or key == ord('q') or cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) == 0.0:
            del arr
            break
        idx += 1

    print('quit')
    cv2.destroyAllWindows()
    cap.stop()

if __name__ == '__main__':
    import sys
    cfg = ""
    if len(sys.argv) > 1:
        cfg = sys.argv[1]
    run_x(cfg)
