import yaml
import cv2
import pickle
import logging
from datetime import datetime
from typing import Dict
import time

import libs.yei3.threespace_api as ts_api
from providers.gps_provider import GPSProvider
from providers.canbus_provider import CanbusProvider
from providers.camera_provider import CameraSharedMemProvider


class Streamer:

    def __init__(self):
        # load settings from configuration file
        with open("config.yaml", "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        if self.settings["enabled_features"]["video"]:
            # assign cameras to their positions and check everything is working
            self.camera_ids = self.settings["camera_ids"]

            self.camera_providers = {}

            # setup the camera providers
            for pos, cam_id in self.camera_ids.items():
                self.camera_providers[pos] = CameraSharedMemProvider(cam_id)

            if self.settings["enabled_features"]["video_undistort"]:
                # load calibration data for all cameras

                self.cam_calib_data = {}

                for position, serial_no in self.settings["camera_serial_numbers"].items():
                    with open("camera/calib_coefs_%s.pkl" % serial_no, "rb") as f:
                        self.cam_calib_data[position] = pickle.load(f)

        # setup GPS
        if self.settings["enabled_features"]["gps"]:
            self.gps_provider = GPSProvider(self.settings["gps_port"])  # TODO auto find port like IMU does?

        # setup IMU
        if self.settings["enabled_features"]["imu"]:
            self.imu_device = self.get_imu_device()

            # TODO is it necessary to set device streaming timer?

            if self.imu_device is None:
                logging.warning("IMU Sensor not found")
            else:
                # TODO POSE: Point position + Quaternion orientation
                # TODO TWIST: Vector3 linear + Vector3 angular velocities
                # TODO IMU MESSAGE: Quaternion orientation + Vector3 angular_velocity + Vector3 linear_acceleration

                # From manual: Corrected Sensor Data:
                # This refers to 'raw' data that has been biased and scaled to represent real-world units
                # For the accelerometer, these values are in units of g-forces,
                # for the magnetometer, these values are in units of gauss,
                # and for the gyroscope, these values are in units of radians/sec

                self.imu_device.setStreamingSlots(
                    slot0='getTaredOrientationAsQuaternion',  # in x, y, z, w order
                    # From manual: "Note that this result is the same data returned by the normalized gyro rate command."
                    slot1='getCorrectedGyroRate',  # in radians / sec
                    slot2='getCorrectedLinearAccelerationInGlobalSpace'  # in G's
                )

                # TODO try a streaming session with a worker process similar to gps and compare performance

            # TODO imu_device.close() when finished

        # setup CAN BUS
        if self.settings["enabled_features"]["canbus"]:
            self.canbus_provider = CanbusProvider(can_device=self.settings["can_device"],
                                                  dbc_file=self.settings["can_dbc_file"])

    def get_imu_device(self) -> ts_api.ComInfo:
        """Gets a reference to the YEI 3 sensor"""
        return ts_api.TSUSBSensor(com_port=self.settings["imu_port"])

    def stream_generator(self) -> Dict[str, object]:
        """Generator that returns timestamped images + sensor data"""

        while True:

            return_packet = {
                "images": {},
                "sensor_data": {},
                "datetime": datetime.now()
            }

            last_time = time.time()

            if self.settings["enabled_features"]["video"]:

                for pos, p_cam in self.camera_providers.items():

                    if p_cam.has_unread_data():
                        frame = p_cam.get_last_frame_raw()  # p_cam.get_last_frame_as_ocv()
                    else:
                        frame = None

                    # if self.settings["enabled_features"]["video_undistort"]:
                    #
                    #     # TODO will only work with RGB!!!!
                    #
                    #     h, w = frame.shape[:2]
                    #
                    #     cameramtx, roi = cv2.getOptimalNewCameraMatrix(
                    #         self.cam_calib_data[pos]['mtx'],
                    #         self.cam_calib_data[pos]['dist'],
                    #         (w, h),
                    #         1,
                    #         (w, h)
                    #     )
                    #
                    #     corrected = cv2.undistort(
                    #         frame,
                    #         self.cam_calib_data[pos]['mtx'],
                    #         self.cam_calib_data[pos]['dist'],
                    #         None,
                    #         cameramtx
                    #     )  # FIXME
                    #
                    #     x, y, w, h = roi
                    #     corrected = corrected[y:y + h, x:x + w]
                    #
                    #     return_packet["images"][pos] = corrected
                    # else:
                    return_packet["images"][pos] = frame

            print("get_from_cams=", time.time() - last_time)
            last_time = time.time()

            # get sensor data

            if self.settings["enabled_features"]["imu"]:

                imu_batch = self.imu_device.getStreamingBatch()

                # TODO this one has no is_data_avail() implemented

                print("get_from_imu=", time.time() - last_time)
                last_time = time.time()

                return_packet["sensor_data"]["imu"] = {
                    "orientation_quaternion": {
                        "x": imu_batch[0],
                        "y": imu_batch[1],
                        "z": imu_batch[2],
                        "w": imu_batch[3]
                    },
                    "gyro_rate": {
                        "x": imu_batch[4],
                        # TODO ensure axes are correctly oriented and run cmds 22 and 20 once installed on the car
                        "y": imu_batch[5],
                        "z": imu_batch[6],
                    },
                    "linear_acceleration": {
                        "x": imu_batch[7],
                        # TODO ensure axes are correctly oriented and run cmds 22 and 20 once installed on the car
                        "y": imu_batch[8],
                        "z": imu_batch[9],
                    },
                }

            print("packet_from_imu=", time.time() - last_time)
            last_time = time.time()

            if self.settings["enabled_features"]["gps"]:

                if self.gps_provider.has_unread_data():
                    gps_msgs = self.gps_provider.get_latest_messages()
                    return_packet["sensor_data"]["gps"] = gps_msgs
                else:
                    return_packet["sensor_data"]["gps"] = None

            print("get_from_gps=", time.time() - last_time)
            last_time = time.time()

            if self.settings["enabled_features"]["canbus"]:

                if self.canbus_provider.has_unread_data():
                    canbus_msgs = self.canbus_provider.get_latest_messages()
                    return_packet["sensor_data"]["canbus"] = canbus_msgs
                else:
                    canbus_msgs = self.canbus_provider.get_latest_messages()
                    return_packet["sensor_data"]["canbus"] = canbus_msgs

            print("get_from_can=", time.time() - last_time)
            last_time = time.time()

            yield return_packet

    def close(self):

        # bring down video providers
        if self.settings["enabled_features"]["video"]:

            for pos, cam_id in self.camera_ids.items():
                self.camera_providers[pos].close()

        # bring down gps
        if self.settings["enabled_features"]["gps"]:
            self.gps_provider.close()  # TODO auto find port like IMU does?

        # bring down IMU
        if self.settings["enabled_features"]["imu"]:
            # TODO
            pass

        # bring down CAN SPEED
        if self.settings["enabled_features"]["canbus"]:
            self.canbus_provider.close()
