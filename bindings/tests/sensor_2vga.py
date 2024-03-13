import os, sys, time, json
import numpy as np
from mayfly.videocapture import VideoCapture
import datetime
import torch
#import jax
#import jax.dlpack

if __name__ == '__main__':
    config = ''
    if len(sys.argv) > 1:
        config=sys.argv[1]

    use_cuda = True
    has_display = os.getenv('DISPLAY') is not None
    if has_display:
        import cv2

    cap = VideoCapture(config)
    idx = 0
    while True:
        frames = cap.read()
        assert len(frames) <= 2, "doesn't work for more than 2 images"
        image = np.zeros([480,1280,3],np.uint8)
        times = [frm['captureTime'] for frm in frames]
        ids = [frm['streamId'] for frm in frames]
        delta_time = max(times) - min(times)
        #print(frames)
        if delta_time > 166:
            print(times, delta_time)

        for frm_idx, frm in enumerate(frames):
            #print('(device, id) = ', frm['image'].__dlpack_device__())

            if use_cuda:
#                tens = jax.dlpack.from_dlpack(frm['image'].__dlpack__())
#                tens = jax.device_put(tens, device=jax.devices("cpu")[0])
#                arr = np.array(tens)
                arr = torch.from_dlpack(frm['image']).cpu().numpy()
            else:
                arr = np.from_dlpack(frm['image'])

            if frm_idx == 0:
                image[:,:640,:] = arr[:,:,:3]
            else:
                image[:,640:,:] = arr[:,:,:3]

        if has_display:
            cv2.imshow('Combined Streams', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        else:
            if idx%1000 == 0:
                print('%s, frame %12d' % (str(datetime.datetime.now()), idx))
            idx+=1

        if has_display:
            key = cv2.waitKey(1) & 0xff
            if key == 27 or key == ord('q'):
                del arr
                break

    example_client_config_pc='''
{
    "id":"client-pc",
    "services":[
        {
            "type":"sensorClient",
            "id":"compute_client",
            "decoder":{
                "type":"nvdecode",
                "device":"cuda"
            },
            "transfers":[
                {
                    "protocol":"udp",
                    "bindAddress":{
                        "ip":"0.0.0.0",
                        "port":6000
                    },
                    "hostAddress":{
                        "ip":"10.10.10.250",
                        "port":9999
                    }
                }
            ]
        }
    ],
    "lastAccess":1673958299458225
}
'''
    example_client_config_247='''
{
    "id":"pi_camera247",
    "services":[
        {
            "type":"videoServer",
            "id":"camera247",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":9999
                }
            },
            "sources":[
                {
                    "id":3,
                    "type":"camera",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":30,
                    "mode":"vga: 640x480"
                }
            ]
        }
    ]
}
'''
    example_client_config_250='''
{
    "id":"pi_camera250",
    "services":[
        {
            "type":"videoServer",
            "id":"camera250",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"10.10.10.250",
                    "port":9990
                }
            },
            "sources":[
                {
                    "type":"camera",
                    "mode":"vga: 640x480",
                    "id":1,
                    "clock":30
                }
            ]
        },
        {
            "type":"videoServer",
            "id":"sync250",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"10.10.10.250",
                    "port":9999
                }
            },
            "sources":[
                {
                    "type":"synccameramaster",
                    "clock":30,
                    "mode":"vga: 640x480",
                    "id":2,
                    "slaves":[
                        {
                            "transfer":{
                                "protocol":"udp",
                                "bindAddress":{
                                    "ip":"10.10.10.250",
                                    "port":5011
                                },
                                "hostAddress":{
                                    "ip":"10.10.10.250",
                                    "port":9990
                                }
                            }
                        },
                        {
                            "transfer":{
                                "protocol":"udp",
                                "bindAddress":{
                                    "ip":"10.10.10.250",
                                    "port":5010
                                },
                                "hostAddress":{
                                    "ip":"10.10.10.247",
                                    "port":9999
                                }
                            }
                        }
                    ]
                }
            ]
        }
    ],
    "lastAccess":1675432514807737
}
'''
    print(example_client_config_pc)
    print(example_client_config_247)
    print(example_client_config_250)
