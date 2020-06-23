import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D

import time
import signal
import libs.yei3.threespace_api as ts_api

filter_flag = ts_api.TSS_FIND_ALL_KNOWN ^ ts_api.TSS_FIND_DNG
device_list = ts_api.getComPorts(filter=filter_flag)

com_port, friendly_name, device_type = device_list[0]

device = None
if device_type == "USB":
    device = ts_api.TSUSBSensor(com_port=com_port)
elif device_type == "WL":
    device = ts_api.TSWLSensor(com_port=com_port)
elif device_type == "EM":
    device = ts_api.TSEMSensor(com_port=com_port)
elif device_type == "DL":
    device = ts_api.TSDLSensor(com_port=com_port)
elif device_type == "BT":
    device = ts_api.TSBTSensor(com_port=com_port)



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

            x_rot[i] = rp[0]
            y_rot[i] = rp[1]
            z_rot[i] = rp[2]

        ax.plot(x_rot, y_rot, z_rot, color='blue', label='rotated')

        plt.legend()

        plt.show(block=False)
        plt.pause(0.0001)

        if terminate:
            break

    ## Now close the port.
    device.close()
