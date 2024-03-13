from mayfly import video
import numpy as np
import cv2
import torch
import tempfile

def write_config(configString):
    tname = tempfile.mkstemp()
    return tname

def show(img, info):
    stream_id, stream_time = info
    cap_time_str = "%6.3f" % (stream_time / 1000)
    cv2.putText(img, cap_time_str, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (250,250,250), 2, 8)
    cv2.imshow('stream_%d'%stream_id, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

def test_read_single_ram():
    cfg='''
{
    "id":"pc0",
    "services":[
        {
            "type":"sensorServer",
            "id":"server0",
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
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":30,
                    "mode":"720p: 1280x720"
                }
            ]
        },
        {
            "type":"sensorClient",
            "id":"client0",
            "errorCorrection":{
                "type":"arq"
            },
            "decoder":{
                "type":"nvdecode",
                "device":"ram_not_cuda"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":6000
                },
                "hostAddress":{
                    "ip":"127.0.0.1",
                    "port":9999
                }
            }
        }
    ]
}
'''
    tmp_file = tempfile.NamedTemporaryFile('w')
    tmp_file.write(cfg)
    tmp_file.flush() # closing is removing
    cap = video.VideoCapture(tmp_file.name)
    for _ in range(100):
        info, frm = cap.read_single_ram()
        img = np.from_dlpack(frm)
        print('info, frm', info, frm)
        #continue
        show(img, info)
        key = cv2.waitKey(10) & 0xff
        if key == 27 or key == ord('q'):
            break
    print('quit %s' % __name__)
    cv2.destroyAllWindows()
    cap.stop()

def test_read_single_cuda():
    cfg='''
{
    "id":"pc0",
    "services":[
        {
            "type":"sensorServer",
            "id":"server0",
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
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":30,
                    "mode":"720p: 1280x720"
                }
            ]
        },
        {
            "type":"sensorClient",
            "id":"client0",
            "errorCorrection":{
                "type":"arq"
            },
            "decoder":{
                "type":"nvdecode",
                "device":"cuda"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":6000
                },
                "hostAddress":{
                    "ip":"127.0.0.1",
                    "port":9999
                }
            }
        }
    ]
}
'''
    tmp_file = tempfile.NamedTemporaryFile('w')
    tmp_file.write(cfg)
    tmp_file.flush() # closing is removing
    cap = video.VideoCapture(tmp_file.name)
    for _ in range(100):
        info, frm = cap.read_single_cuda()
        print('info, frm', info, frm)
        #continue
        img = torch.from_dlpack(frm).cpu().numpy()
        show(img, info)
        key = cv2.waitKey(10) & 0xff
        if key == 27 or key == ord('q'):
            break
    print('quit %s' % __name__)
    cv2.destroyAllWindows()
    cap.stop()

def test_read_combined_ram():
    cfg='''
{
    "id":"pc0",
    "services":[
        {
            "type":"sensorServer",
            "id":"server0",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":9997
                }
            },
            "sources":[
                {
                    "id":3,
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":15
                }
            ]
        },
        {
            "type":"sensorServer",
            "id":"server1",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":9998
                }
            },
            "sources":[
                {
                    "id":4,
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":15
                }
            ]
        },
        {
            "type":"sensorServer",
            "id":"server2",
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
                    "type":"synccameramaster",
                    "id":5,
                    "clock":15,
                    "slaves":[
                        {
                            "transfer":{
                                "protocol":"udp",
                                "bindAddress":{
                                    "ip":"0.0.0.0",
                                    "port":5011
                                },
                                "hostAddress":{
                                    "ip":"127.0.0.1",
                                    "port":9997
                                }
                            }
                        },
                        {
                            "transfer":{
                                "protocol":"udp",
                                "bindAddress":{
                                    "ip":"0.0.0.0",
                                    "port":5012
                                },
                                "hostAddress":{
                                    "ip":"127.0.0.1",
                                    "port":9998
                                }
                            }
                        }
                    ]
                }
            ]
        },
        {
            "type":"sensorClient",
            "id":"client0",
            "errorCorrection":{
                "type":"arq"
            },
            "decoder":{
                "type":"nvdecode",
                "device":"ram_not_cuda"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":6000
                },
                "hostAddress":{
                    "ip":"127.0.0.1",
                    "port":9999
                }
            }
        }
    ]
}
'''
    tmp_file = tempfile.NamedTemporaryFile('w')
    tmp_file.write(cfg)
    tmp_file.flush() # closing is removing
    cap = video.VideoCapture(tmp_file.name)
    for _ in range(100):
        infos, frms = cap.read_combined_ram()
        for info, frm in zip(infos, frms):
            img = np.from_dlpack(frm)
            print('info, frm', info, frm)
            #continue
            show(img, info)
        #continue
        key = cv2.waitKey(10) & 0xff
        if key == 27 or key == ord('q'):
            break
    print('quit %s' % __name__)
    cv2.destroyAllWindows()
    cap.stop()

def test_read_combined_cuda():
    cfg='''
{
    "id":"pc0",
    "services":[
        {
            "type":"sensorServer",
            "id":"server0",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":9997
                }
            },
            "sources":[
                {
                    "id":3,
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":15
                }
            ]
        },
        {
            "type":"sensorServer",
            "id":"server1",
            "errorCorrection":{
                "type":"arq"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":9998
                }
            },
            "sources":[
                {
                    "id":4,
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":15
                }
            ]
        },
        {
            "type":"sensorServer",
            "id":"server2",
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
                    "type":"synccameramaster",
                    "id":5,
                    "clock":15,
                    "slaves":[
                        {
                            "transfer":{
                                "protocol":"udp",
                                "bindAddress":{
                                    "ip":"0.0.0.0",
                                    "port":5011
                                },
                                "hostAddress":{
                                    "ip":"127.0.0.1",
                                    "port":9997
                                }
                            }
                        },
                        {
                            "transfer":{
                                "protocol":"udp",
                                "bindAddress":{
                                    "ip":"0.0.0.0",
                                    "port":5012
                                },
                                "hostAddress":{
                                    "ip":"127.0.0.1",
                                    "port":9998
                                }
                            }
                        }
                    ]
                }
            ]
        },
        {
            "type":"sensorClient",
            "id":"client0",
            "errorCorrection":{
                "type":"arq"
            },
            "decoder":{
                "type":"nvdecode",
                "device":"cuda"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":6000
                },
                "hostAddress":{
                    "ip":"127.0.0.1",
                    "port":9999
                }
            }
        }
    ]
}
'''
    tmp_file = tempfile.NamedTemporaryFile('w')
    tmp_file.write(cfg)
    tmp_file.flush() # closing is removing
    cap = video.VideoCapture(tmp_file.name)
    for _ in range(100):
        infos, frms = cap.read_combined_cuda()
        for info, frm in zip(infos, frms):
            img = torch.from_dlpack(frm).cpu().numpy()
            print('info, frm', info, frm)
            #continue
            show(img, info)
        #continue
        key = cv2.waitKey(10) & 0xff
        if key == 27 or key == ord('q'):
            break
    print('quit %s' % __name__)
    cv2.destroyAllWindows()
    cap.stop()


if __name__ == '__main__':
    test_read_single_ram()
    test_read_combined_ram()
    test_read_single_cuda()
    test_read_combined_cuda()
