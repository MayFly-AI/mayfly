''' Example demonstrating pytorch processing of video decoded in CUDA memory
    we're using PyTorch to process the image, package available from pytorch.org '''
import os
import json
import tempfile
import argparse
from mayfly.sensorcapture import SensorCapture
import torch

CLIENT_CONFIG = '''
{
    "id":"example_client",
    "services":[
        {
            "type":"sensorClient",
            "id":"cuda_video_client",
            "decoder":{"type":"nvdecode", "device":"cuda"},
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

def process_camera(video_source):
    ''' Uses pytorch to calculate the red, green, and blue mean values for each frame '''
    frames = video_source.get('frames', [])
    for frame in frames:
        tensor = torch.from_dlpack(frame['image'])

        mean_red = torch.mean(tensor[:,:,0]/255.0)
        mean_green = torch.mean(tensor[:,:,1]/255.0)
        mean_blue = torch.mean(tensor[:,:,2]/255.0)
        print('video mean RGB: %4.2f, %4.2f, %4.2f' % (mean_red, mean_green, mean_blue))


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
        if sensor_data.get('type','') == 'camera':
            process_camera(sensor_data)
