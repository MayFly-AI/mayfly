''' Example demonstrating receiving sensor data from a sensor server '''
import os
import json
import tempfile
import argparse
from mayfly.sensorcapture import SensorCapture

CLIENT_CONFIG = '''
{
    "id":"example_client",
    "services":[
        {
            "type":"sensorClient",
            "id":"print_sensors_client",
            "decoder":{"type":"openh264"},
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
