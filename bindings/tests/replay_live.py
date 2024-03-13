import os, json, glob
from os.path import join as pjoin
import numpy as np
import quat_compass
import glm
import scipy
import cv2
from mayfly import sensor

class ImuMagAggregator:
    def __init__(self):
        self.agg = {}
        self.mag_bias = np.loadtxt('mag_bias.txt')
        print(self.mag_bias)

    def add_source(self,src_):
        src_type = src_.get('type', '')
        if src_type == 'imu':
            self.add_imu(src_)
        if src_type == 'mag':
            self.add_mag(src_)

    def add_imu(self,src_):
        assert len(src_['values']['acc']) == 3, "Multiple IMU measurements are not supported"
        self.agg['acc'] = src_['values']['acc'][-3:]
        self.agg['gyr'] = src_['values']['gyr'][-3:]
        self.agg['t_imu'] = src_['values']['time'][-1]

    def add_mag(self,src_):
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
    def __init__(self):
        with open('bases.json') as fh:
            self.bases = json.load(fh)
            self.id_pos = {b['id']:b['pos'] for b in self.bases}
        self.history = []
        self.last_pos = None

    def add_source(self,src_):
        src_type = src_.get('type', '')
        if src_type != 'tag':
            return
        values = src_.get('values', [])
        if not values:
            return
        time_us = src_['time']
        for value in values:
            bases = value.get('bases', [])
            id_dist = {base['id']: (base['dist'], base['nlos']) for base in bases}
            if bases:
                self.history.append((time_us, id_dist))

    def find_position(self):
        if not self.history:
            return None

        time, id_dist_nlos = self.history[-1]
        base_positions = [self.id_pos[base_id] for base_id in id_dist_nlos.keys()]
        base_distances = [d[0] for d in id_dist_nlos.values()]
        base_nloses = [d[1] for d in id_dist_nlos.values()]

        base_positions = np.array(base_positions, np.float32)
        base_distances = np.array(base_distances)
        weights = 1.0 - np.array(base_nloses)

        guess_pos = np.mean(base_positions, axis=0)

        assert len(base_distances) == len(base_positions)
        assert len(base_distances) == len(weights)

        def loss(p_, base_distances_, base_positions_, weights_):
            l2 = 0.0
            for d, pb, w in zip(base_distances_, base_positions_, weights_):
                l2 += w*((p_[0] - pb[0])**2 + (p_[1] - pb[1])**2 - d**2)**2
            return l2

        tolerance = 0.1
        res = scipy.optimize.minimize(loss, guess_pos, args=(base_distances, base_positions, weights), tol=tolerance)
        best_loss_val = res['fun'] / len(base_distances)
        guess_pos = None
        if not res.success:
            print('OPTIMIZER FAILED', res)
        else:
            guess_pos = res.x
            self.last_pos = res.x

        return guess_pos

    def draw(self):
        wx_off = -4
        wy_off = -3
        scale = 10/500
        def to_pixel(wx, wy):
            px = (wx - wx_off)/scale
            py = (wy - wy_off)/scale
            return int(round(px)), int(round(py))

        self.bgr = np.zeros([480,640,3], np.uint8)
        for bs in self.bases:
            pos = to_pixel(bs['pos'][0], bs['pos'][1])
            cv2.circle(self.bgr, pos, 7, bs['color'], -1, cv2.LINE_AA)
            cv2.circle(self.bgr, pos, 10, bs['color'], 1, cv2.LINE_AA)
            cv2.putText(self.bgr, bs['name'], (pos[0]+15, pos[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250, 250, 250), 1, cv2.LINE_AA)
        if self.last_pos is not None:
            pos = to_pixel(self.last_pos[0], self.last_pos[1])
            cv2.circle(self.bgr, pos, 7, (12,32,300), -1, cv2.LINE_AA)

        return self.bgr

def main_replay_live(directory, config_filename):
    compass = quat_compass.QuatCompass()
    agg = ImuMagAggregator()
    tri = Trilateration()

    tag_dist = {}
    base_dists = {}
    mag_all = []
    rot = None

    ALL_STREAMS=list(range(60))
    app = sensor.AppSensor()
    queue = app.create_event_queue(ALL_STREAMS, 4)
    app.start(config_filename)

    while True:
        res = queue.wait()
        if res is None:
            continue

        src = res['source']

        images = []
        for frm in src.get('frames',[]):
            if '$data' in frm:
                name = frm['$data']['name']
                index = frm['$data']['idx']
                frm[name] = np.from_dlpack(res['$data'][index])
                images.append(frm[name])

        tri.add_source(src)
        tri.find_position()
        tri_frame = tri.draw()

        agg.add_source(src)
        imu_mag = agg.read()
        if imu_mag is not None:
            acc, gyr, mag, dt = imu_mag
            res = compass.update(acc, gyr, mag, dt)
            rot = glm.mat3_cast(res)
            mag_all.append(mag.tolist())

        for img in images:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if rot is not None:
                cv2.circle(bgr, (100,100), 55, (250,250,250), 1, cv2.LINE_AA)
                cv2.putText(bgr, 'N', (95, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 250, 250), 1, cv2.LINE_AA)
                cv2.line(bgr, (100,100), (100+int(50*rot[0][1]), 100-int(50*rot[0][0])), (250,250,255), 2, cv2.LINE_AA)
            cv2.imshow('video', bgr)

        cv2.imshow('trilateration', tri_frame)
        key = cv2.waitKey(1) & 0xff
        if key == 27 or key == ord('q'):
            break

    print('done replaying')
    cv2.destroyAllWindows()
    app.stop()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', type=str, required=False, help='record directory')
    parser.add_argument('--config', type=str, required=True, help='config file')
    args = parser.parse_args()

    main_replay_live(args.dir, args.config)
