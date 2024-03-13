import numpy as np
import glm

class QuatCompass:
    """
        Complementary filtering of orientation based on quaternions.

        1) The accelerometer and the magnetometer data is used to calculate the
            North,East,Down frame of reference.
        2) The gyro is applied relative to the previous orientation.

        The two estimates are spherically interpolated using slerp to update the orientation.
    """

    def __init__(self):
        self.model_rotation = np.array([1,0,0,0], np.float32)
        self.prev_rotation = None
        self.alpha = 0.7

    # acc : np.array([3])
    # gyr : np.array([3])
    # mag : np.array([3])
    # dt  : delta time in seconds
    def update(self, acc, gyr, mag, dt):
        mag = np.array([-mag[1], -mag[0], -mag[2]]) # -> imu coordinates

        ned = self.calc_ned(acc, mag)
        q_imu = glm.quat_cast(ned)
        q_imu = q_imu/np.linalg.norm(q_imu)

        if self.prev_rotation is None:
            self.prev_rotation = q_imu

        q_gyro = self.gyro_integration(self.prev_rotation, gyr, dt)
        rotation = glm.slerp(q_imu, q_gyro, self.alpha)

        # antenna (N), swd (E), pcb (D)
        ref = np.array([[0,1,0], [1,0,0], [0,0,-1]], dtype=np.float32)
        qref = glm.quat_cast(ref)
        self.model_rotation = glm.inverse(qref) * rotation
        self.prev_rotation = rotation
        return self.model_rotation

    def calc_ned(self, acc_, mag_):
        """ Calculates the orthogonal North,East,Down directions in local coordinates """
        down = -acc_/np.linalg.norm(acc_)
        east = np.cross(down, mag_)
        east = east / np.linalg.norm(east)
        north = np.cross(east, down)
        return np.stack([north, east, down]).astype(np.float32)

    def gyro_integration(self, rot_, gyr_, dt_):
        """ Applies the gyro motion to the previous rotation """
        L = glm.quat(0, gyr_[0], gyr_[1], gyr_[2])
        grot = rot_ + 0.5*dt_*rot_*L
        grot = grot / np.linalg.norm(grot)
        return grot
