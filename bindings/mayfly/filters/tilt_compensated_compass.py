import numpy as np

class TiltCompensatedCompass:
    def __init__(self):
        pass

    # acc : np.array([3])
    # mag : np.array([3])
    def update(self, acc, mag):
        ax, ay, az = acc
        mx, my, mz = mag

        ex = np.arctan2( ay, az)                        # Roll (around x)
        ey = np.arctan2(-ax, np.sqrt(ay**2 + az**2))    # Pitch (around y)

        mag = mag/np.linalg.norm(mag) # normalize

        # Switch mag coord system to fit IMU
        mx_ = -mag[1]
        my_ = -mag[0]
        mz_ = -mag[2]

        # Get tilted reference frame
        by = my_*np.cos(ex) - mz_*np.sin(ex)
        bx = mx_*np.cos(ey) + np.sin(ey)*(my_*np.sin(ex) + mz_*np.cos(ex))
        ez = np.arctan2(-by, bx)

        return ex, ey, ez, bx, by
