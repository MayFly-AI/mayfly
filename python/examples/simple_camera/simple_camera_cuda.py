''' Example demonstrating receiving sensor data from a sensor server '''
import os
import json
import tempfile
import argparse
import numpy as np
import cv2
import torch
from mayfly.sensorcapture import SensorCapture

CLIENT_CONFIG = '''
{
    "id":"example_client",
    "services":[
        {
            "type":"sensorClient",
            "id":"print_sensors_client",
            "decoder":{"type":"nvdecode"},
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, required=True,
                        help='IP address of the sensor server')
    parser.add_argument('--port', type=int, default=8999, required=False,
                        help='port number of the sensor server')
    args = parser.parse_args()

    cfg_filename = configure(CLIENT_CONFIG, args.ip, args.port)

    cap = SensorCapture(cfg_filename)
    while True:
        sensor_data = cap.read()
        print(sensor_data)
        if sensor_data['type'] == 'camera':
            frm = sensor_data['frames'][0] # It may have more than 1 frame if sync cameras or ToF. We assume 1 frame
            arr = torch.from_dlpack(frm['image']).cpu().numpy()
            arr = cv2.cvtColor(arr[:,:,:3], cv2.COLOR_RGB2BGR)
            cv2.imshow('Stream from '+args.ip, arr)
            cv2.waitKey(1)

