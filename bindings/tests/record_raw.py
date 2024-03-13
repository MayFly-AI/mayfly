import os
import json
from os.path import join as pjoin
import numpy as np
from mayfly import sensor
import h264decoder

show_video = True
import cv2

class VideoDecoder:
    def __init__(self):
        self.decoder = h264decoder.H264Decoder()
    def decode(self, bin_data):
        images = []
        res = self.decoder.decode(bin_data)
        for frame, w, h, ls in res:
            frame = np.frombuffer(frame, dtype=np.ubyte, count=len(frame))
            frame = frame.reshape((h, ls//3, 3))
            images.append(frame[:,:w,:])
        return images

def main_record_raw(directory, config_filename):
    os.makedirs(directory, exist_ok=False)

    ALL_STREAMS=list(range(60))
    app = sensor.AppSensor()
    queue = app.create_event_queue(ALL_STREAMS, 4)
    app.start(config_filename)

    dec = VideoDecoder()

    read_index = 0
    MAX_CAPTURE_COUNT = 50000
    while read_index < MAX_CAPTURE_COUNT:
        res = queue.wait(3)
        if res is None:
            continue

        print(res)
        if '$data' in res:
            for tid, t in enumerate(res['$data']):
                assert isinstance(t, sensor.Tensor), 'Expected type tensor'
                arr = np.from_dlpack(t).copy()
                arr_name = 'data_%06d_%03d.bin' % (read_index, tid)
                arr.tofile(pjoin(directory,arr_name))
                if show_video:
                    images = dec.decode(arr.tobytes())
                    if images:
                        cv2.imshow('video', cv2.cvtColor(images[-1], cv2.COLOR_RGB2BGR))
                        cv2.waitKey(1)

            del res['$data']

        if 'source' in res:
            src = res['source']
            for frm in src.get('frames',[]):
                if '$data' in frm:
                    data_idx = frm['$data']['idx']
                    arr_name = 'data_%06d_%03d.bin' % (read_index, data_idx)
                    frm['$data']['filename']=arr_name
            src_name=pjoin(directory, 'source_%06d.json' % read_index)
            with open(src_name, 'w') as fh:
                json.dump(src, fh, indent=2)

        read_index+=1

    app.stop()
    print('main_record_raw done')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, required=True, help='record directory')
    parser.add_argument('--config', type=str, required=False, help='config file')
    args = parser.parse_args()

    rec_dir = args.dir
    cfg_file = args.config
    main_record_raw(rec_dir, cfg_file)
