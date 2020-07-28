import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D

import time
import signal
import libs.yei3.threespace_api as ts_api


device = ts_api.TSUSBSensor(com_port="/dev/ttyACM0")

terminate = False

def _handle_signal(signum, frame):
    global terminate
    terminate = True


signal.signal(signal.SIGINT, _handle_signal)

# s = np.linspace(0,2*np.pi,100)
# x = np.cos(s)
# y = np.sin(s)
# z = np.zeros_like(x)
# plt.plot(x, y, zs=0, zdir='z', label='transformation with Quaternions')
# plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# ax.plot((0,0),(0,0), (-10,10), '-k', label='z-axis')
# ax.legend()

theta = np.linspace(0, 2 * np.pi, 201)
x = 10*np.cos(theta)
y = 10*np.sin(theta)
z = np.zeros_like(x)

x_rot = np.zeros_like(x)
y_rot = np.zeros_like(y)
z_rot = np.zeros_like(z)

if device is not None:
    print(device)

    device.setStreamingSlots(
        slot0='getTaredOrientationAsQuaternion',
        # From manual: "Note that this result is the same data returned by the normalized gyro rate command."
        # slot1='getCorrectedGyroRate',  # in radians / sec
        # slot2='getCorrectedLinearAccelerationInGlobalSpace'  # in G's
    )

    while True:
        batch = device.getStreamingBatch()

        print(batch)

        # x_batch, y_batch, z_batch, w_batch = batch

        y_batch, x_batch, z_batch, w_batch = batch

        crt_quat = Quaternion(w_batch, x_batch, y_batch, z_batch)

        plt.cla()

        ax.plot(x, y, z, color='red', label='reference')

        for i in range(x.shape[0]):
            rp = crt_quat.rotate([x[i], y[i], z[i]])

            x_rot[i] = rp[1]
            y_rot[i] = rp[0]
            z_rot[i] = rp[2]

        ax.plot(x_rot, y_rot, z_rot, color='blue', label='rotated')

        plt.legend()

        plt.show(block=False)
        plt.pause(0.0001)

        if terminate:
            break

    ## Now close the port.
    device.close()
