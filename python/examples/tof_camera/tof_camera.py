''' Example demonstrating receiving and showing ToF sensor data from a sensor server
    we're using OpenCV to display the depth image, avaliable with the opencv-python package '''
import os
import json
import tempfile
import argparse
import numpy as np
from mayfly.sensorcapture import SensorCapture
import cv2

CLIENT_CONFIG = '''
{
    "id":"example_client",
    "services":[
        {
            "type":"sensorClient",
            "id":"tof_camera",
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":6000
                },
                "hostAddress":{
                    "ip":"0.0.0.0",
                    "port":8999
                }
            }
        }
    ]
}
'''

def configure(config_str, ip_address, port_number):
    ''' modifies a simple configuration file for the sensor client to be able to
        receive data from the sensor server available at the specified IP address and port number
    '''

    config = json.loads(config_str)

    config['services'][0]['transfer']['hostAddress']['ip'] = ip_address
    config['services'][0]['transfer']['hostAddress']['port'] = port_number
    mod_config = json.dumps(config, indent=2)

    fd, filename = tempfile.mkstemp('_mayfly.json')
    print(f'writing config file: {filename}')
    os.write(fd, bytes(mod_config, 'utf-8'))
    os.close(fd)
    return filename

def process_tof(tof_source, max_depth=4.0):
    ''' extracts the ToF data normalizes it and displays it using OpenCV '''
    frames = tof_source.get('frames',[])
    for idx, frame in enumerate(frames):
        if 'depth' in frame:
            depth_image = np.from_dlpack(frame['depth']) / max_depth
            cv2.imshow('ToF Camera %d' % idx, depth_image)

    key = cv2.waitKey(1) & 0xff
    return key == ord('q') or key == 27

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, required=True,
                        help='IP address of the sensor server')
    parser.add_argument('--port', type=int, default=8999, required=False,
                        help='port number of the sensor server')
    args = parser.parse_args()

    cfg_filename = configure(CLIENT_CONFIG, args.ip, args.port)

    cap = SensorCapture(cfg_filename)

    done = False
    while not done:
        sensor_data = cap.read()
        if sensor_data.get('type','') == 'tof':
            done = process_tof(sensor_data)

    cv2.destroyAllWindows()
