import os, sys, time, json
import numpy as np
import cv2
#from videocapture import VideoCapture
from mayfly.videocapture import VideoCapture
import turbo_colormap

if __name__ == '__main__':
    config = ''
    if len(sys.argv) > 1:
        config=sys.argv[1]

    has_display = False#os.getenv('DISPLAY') is not None
    has_display = True
    cap = VideoCapture(list(range(64)), config)
    while True:
        capture = cap.read()
        print(capture)
        #continue
        for streams in capture['streams']:
            arr = np.from_dlpack(streams['frame'])

            if len(arr.shape)==3:
                image = arr
            elif len(arr.shape)==2:
                image = turbo_colormap.interpolate(turbo_colormap.turbo_colormap_data, 1.0-arr/4.0, normalize=False)
                image = image.astype(np.float32)
            else:
                print('Unhandled image shape:', arr.shape)

            if has_display:
                #image = cv2.flip(image, 0)
                #image = image[::-1,:]
                #image = cv2.flip(image, 1)
                cv2.imshow('StreamID: %d' % streams['id'], cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
            else:
                print('got frame')

        if has_display:
            key = cv2.waitKey(1) & 0xff
            if key == 27 or key == ord('q'):
                del arr
                break

config_file_pc='''
{
    "id":"test-pc",
    "services":[
        {
            "type":"sensorClient",
            "id":"compute_client",
            "transfers":[
                {
                    "protocol":"udp",
                    "bindAddress":{
                        "ip":"0.0.0.0",
                        "port":6000
                    },
                    "hostAddress":{
                        "ip":"10.10.10.246",
                        "port":9999
                    }
                }
            ]
        }
    ]
}
'''

config_file_pi='''
{
    "id":"camera246",
    "httpserverX":{
        "bindAddress":{
            "ip":"10.10.10.246",
            "port":8880
        }
    },
    "debugTransfer":{
        "protocol":"udp",
        "bindAddress":{
            "ip":"10.10.10.246",
            "port":7777
        }
    },
    "services":[
        {
            "type":"sensorServer",
            "id":"camera246_server0",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"10.10.10.246",
                    "port":9999
                }
            },
            "sources":[
                {
                    "type":"tof",
                    "id":21,
                    "clock":30
                },
                {
                    "type":"status",
                    "id":33,
                    "clockDivider":1
                }
            ]
        }
    ]
}
'''
