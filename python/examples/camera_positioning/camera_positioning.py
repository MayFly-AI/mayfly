import os
import json
import tempfile
import argparse
import glm
import numpy as np
from mayfly.filters import quat_compass
from mayfly.sensorcapture import SensorCapture

import cv2
import scipy

CLIENT_CONFIG = '''
{
    "id":"example_client",
    "services":[
        {
            "type":"sensorClient",
            "id":"camera_positioning_client",
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

class ImuMagAggregator:
    def __init__(self, bias_file):
        self.agg = {}
#        self.mag_bias = np.array([0, 0, 0], np.float32)
        with open(bias_file) as fh:
            self.mag_bias = np.array(json.load(fh))
        print('Using magnetic bias ', self.mag_bias, 'from: %s' % bias_file)

    def add_source(self, src_):
        src_type = src_.get('type', '')
        if src_type == 'imu':
            self.add_imu(src_)
        if src_type == 'mag':
            self.add_mag(src_)

    def add_imu(self, src_):
        assert len(src_['values']['acc']) == 3, "Multiple IMU measurements are not supported"
        self.agg['acc'] = src_['values']['acc'][-3:]
        self.agg['gyr'] = src_['values']['gyr'][-3:]
        self.agg['t_imu'] = src_['values']['time'][-1]

    def add_mag(self, src_):
        self.agg['mag'] = src_['values']['mag'][-3:] - self.mag_bias
        self.agg['t_mag'] = src_['values']['time'][-1]

    def read(self):
        if 'acc' in self.agg and 'gyr' in self.agg and 'mag' in self.agg:
            acc = np.array(self.agg['acc'])
            gyr = np.array(self.agg['gyr'])
            mag = np.array(self.agg['mag'])

            del self.agg['acc']
            del self.agg['gyr']
            del self.agg['mag']

            tnow = self.agg['t_imu']
            tprev = self.agg.get('ptime', tnow)
            self.agg['ptime'] = tnow

            dt = (tnow - tprev) / 1e6
            return [acc, gyr, mag, dt]
        return None

class Trilateration:
    def __init__(self, base_measurements):
        with open(base_measurements) as fh:
            self.bases = json.load(fh)
            self.id_pos = {b['id']:b['pos'] for b in self.bases}
            self.known_base_ids = [base['id'] for base in self.bases]
        self.history = []
        self.last_pos = None

    def add_source(self, src_):
        src_type = src_.get('type', '')
        if src_type != 'tag':
            return
        values = src_.get('values', [])
        if not values:
            return
        time_us = src_['time']
        for value in values:
            bases = value.get('bases', [])

            id_dist = {}
            for base in bases:
                base_id = base['id']
                if base_id not in self.known_base_ids:
                    continue
                id_dist[base_id] =  (base['dist'], base['nlos'])
            if id_dist:
                self.history.append((time_us, id_dist))

    def update_position(self):
        if not self.history:
            return None

        time, id_dist_nlos = self.history[-1]
        if not id_dist_nlos:
            return None

        base_positions = [self.id_pos[base_id] for base_id in id_dist_nlos.keys()]
        base_distances = [d[0] for d in id_dist_nlos.values()]
        base_nloses = [d[1] for d in id_dist_nlos.values()]

        base_positions = np.array(base_positions, np.float32)
        base_distances = np.array(base_distances)
        weights = 1.0 - np.array(base_nloses)

        if self.last_pos is None:
            guess_pos = np.mean(base_positions, axis=0)
        else:
            guess_pos = self.last_pos

        assert len(base_distances) == len(base_positions)
        assert len(base_distances) == len(weights)

        def loss(p_, base_distances_, base_positions_, weights_):
            l2 = 0.0
            for d, pb, w in zip(base_distances_, base_positions_, weights_):
                l2 += w*((p_[0] - pb[0])**2 + (p_[1] - pb[1])**2 - d**2)**2
            return l2

        tolerance = 0.1
        res = scipy.optimize.minimize(loss, guess_pos, args=(base_distances, base_positions, weights), tol=tolerance)
        if not res.success:
            print('OPTIMIZER FAILED', res)
        else:
            self.last_pos = res.x

    def draw(self):
        window_size = (640, 480) # width, height
        pad = min(window_size)/5
        base_positions = np.array([bs['pos'] for bs in self.bases])
        min_pos = np.min(base_positions, axis=1)
        max_pos = np.max(base_positions, axis=1)
        scale = np.amax(max_pos[:2] - min_pos[:2]) / (min(window_size)-2*pad)

        def to_pixel(wx, wy):
            px = pad + (wx - min_pos[0])/scale
            py = window_size[1] - pad - (wy - min_pos[1])/scale
            return int(round(px)), int(round(py))

        self.bgr = np.zeros([window_size[1], window_size[0], 3], np.uint8)
        for bs in self.bases:
            pos = to_pixel(bs['pos'][0], bs['pos'][1])
            cv2.circle(self.bgr, pos, 7, bs['color'], -1, cv2.LINE_AA)
            cv2.circle(self.bgr, pos, 10, bs['color'], 1, cv2.LINE_AA)
            cv2.putText(self.bgr, bs['name'], (pos[0]+15, pos[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250, 250, 250), 1, cv2.LINE_AA)
        if self.last_pos is not None:
            pos = to_pixel(self.last_pos[0], self.last_pos[1])
            cv2.circle(self.bgr, pos, 7, (12, 32, 300), -1, cv2.LINE_AA)

        return self.bgr


def main_replay_live(config_filename):
    agg = ImuMagAggregator('magnetometer_bias.json')
    tri = Trilateration('base_measurements.json')

    cap = SensorCapture(config_filename)
    compass = quat_compass.QuatCompass()
    heading = None
    while True:
        sensor_data = cap.read()

        images = []
        for frame in sensor_data.get('frames', []):
            if 'image' in frame:
                image_show = np.from_dlpack(frame['image']).copy()
                images.append(image_show)

        tri.add_source(sensor_data)
        tri.update_position()
        tri_frame = tri.draw()

        agg.add_source(sensor_data)
        imu_mag = agg.read()
        if imu_mag is not None:
            acc, gyr, mag, dt = imu_mag
            res = compass.update(acc, gyr, mag, dt)
            heading = glm.mat3_cast(res)
        for img in images:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if heading is not None:
                cv2.circle(bgr, (100, 100), 55, (250, 250, 250), 1, cv2.LINE_AA)
                cv2.putText(bgr, 'N', (95, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 250, 250), 1, cv2.LINE_AA)
                cv2.line(bgr, (100, 100), (100+int(50*heading[0][1]), 100-int(50*heading[0][0])), (250, 250, 255), 2, cv2.LINE_AA)
            cv2.imshow('video', bgr)

        cv2.imshow('trilateration', tri_frame)
        key = cv2.waitKey(1) & 0xff
        if key == 27 or key == ord('q'):
            break

    print('done replaying')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, required=True,
                        help='IP address of the sensor server')
    parser.add_argument('--port', type=int, default=8999, required=False,
                        help='port number of the sensor server')
    args = parser.parse_args()
    cfg_filename = configure(CLIENT_CONFIG, args.ip, args.port)

    main_replay_live(cfg_filename)
