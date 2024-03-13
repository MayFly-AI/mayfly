import os, sys, time, json
import numpy as np
import cv2
from mayfly.videocapture import VideoCapture
import datetime
import torch
import turbo_colormap
#import jax
#import jax.dlpack

if __name__ == '__main__':
    config = ''
    if len(sys.argv) > 1:
        config=sys.argv[1]

    use_cuda = True
    has_display = os.getenv('DISPLAY') is not None

    cap = VideoCapture(list(range(64)), config)
    idx = 0
    while True:
        frames = cap.read()
        for frm in frames:
            #print('(device, id) = ', frm['image'].__dlpack_device__())
            print(frm)

            if 'image' not in frm:
                print('frames = ',frames)
                continue
            if use_cuda:
#                tens = jax.dlpack.from_dlpack(frm['image'].__dlpack__())
#                tens = jax.device_put(tens, device=jax.devices("cpu")[0])
#                arr = np.array(tens)
                arr = torch.from_dlpack(frm['image']).cpu().numpy()
            else:
                arr = np.from_dlpack(frm['image']).copy()

            if has_display:
                if len(arr.shape)==3:
                    image = arr[:,:,:3]
                elif len(arr.shape)==2:
                    image = turbo_colormap.interpolate(turbo_colormap.turbo_colormap_data, 1.0-arr/4.0, normalize=False)
                    image = image.astype(np.float32)
                    #cv2.imshow('StreamID: %d' % frm['streamId'], arr[:,:])
                    #cv2.imshow('StreamID: %d' % frm['streamId'], cv2.cvtColor(arr[:,:,:3], cv2.COLOR_RGB2BGR))
                else:
                    pass
                cv2.imshow('StreamID: %d' % frm['streamId'], cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            else:
                if idx%1000 == 0:
                    print('%s, frame %12d' % (str(datetime.datetime.now()), idx))
                idx+=1

        if has_display:
            key = cv2.waitKey(1) & 0xff
            if key == 27 or key == ord('q'):
                break
    if has_display:
        cv2.destroyAllWindows()
