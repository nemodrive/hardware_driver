import yaml
import numpy as np
import cv2
import pickle
import logging
from datetime import datetime
from typing import Dict

import libs.yei3.threespace_api as ts_api
from gps_provider import GPSProvider

try:
    from vimba import *
except:
    logging.warning("import vimba failed, video will be unavailable")


class SimpleStreamer:

    def __init__(self):
        # load settings from configuration file
        with open("config.yaml", "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        if self.settings["enabled_features"]["video"]:
            # assign cameras to their positions and check everything is working
            self.cameras = self._get_cameras()

            self.frame_generators = {}

            # setup the cameras by sending vimba commands
            for pos, cam in self.cameras.items():
                self._setup_camera(cam)
                self.frame_generators[pos] = self._camera_frame_generator_wrapper(cam)

        # setup GPS
        self.gps_provider = GPSProvider(self.settings["gps_port"])  # TODO auto find port like IMU does?

        # TODO gps_provider.close() when finished

        # setup IMU
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


    def get_imu_device(self) -> ts_api.ComInfo:
        """Gets a reference to the first YEI 3 sensor found"""

        filter_flag = ts_api.TSS_FIND_ALL_KNOWN ^ ts_api.TSS_FIND_DNG
        device_list = ts_api.getComPorts(filter=filter_flag)

        com_port, friendly_name, device_type = device_list[0]

        logging.info("Found YEI 3 IMU Sensor with port=%s name=%s type=%s", com_port, friendly_name, device_type)

        if device_type == "USB":
            return ts_api.TSUSBSensor(com_port=com_port)
        elif device_type == "WL":
            return ts_api.TSWLSensor(com_port=com_port)
        elif device_type == "EM":
            return ts_api.TSEMSensor(com_port=com_port)
        elif device_type == "DL":
            return ts_api.TSDLSensor(com_port=com_port)
        elif device_type == "BT":
            return ts_api.TSBTSensor(com_port=com_port)
        else:
            return None

    def _get_cameras(self) -> Dict[str, Camera]:
        """Filters connected cameras for the desired serial numbers and assigns them to
        the positions specified in config.yaml
        """

        with Vimba.get_instance() as vi:
            available_cams = vi.get_all_cameras()

            logging.debug("Found %d connected cameras", len(available_cams))

            cameras = {}

            for i, cam in enumerate(available_cams):
                logging.debug("Camera<id=%s, name=%s, model=%s, serial_no=%s, interface_id=%s>",
                              cam.get_id(), cam.get_name(), cam.get_model(), cam.get_serial(), cam.get_interface_id())

                for position, serial_no in self.settings["camera_serial_numbers"].items():
                    if cam.get_serial() == serial_no:
                        logging.debug("Assigning Camera %s to position %s", cam.get_id(), position)
                        cameras[position] = cam

            if not all(cam in cameras.keys() for cam in self.settings["camera_serial_numbers"].keys()):
                logging.warning("Some cameras have not been found!")

            return cameras

    def _setup_camera(self, camera: Camera) -> None:
        """Set camera parameters before starting frame capture"""
        with Vimba.get_instance():
            with camera:
                if self.settings["camera_auto_features"]["auto_adjust_packet_size"]:
                    try:
                        camera.GVSPAdjustPacketSize.run()
                        while not camera.GVSPAdjustPacketSize.is_done():
                            pass
                    except (AttributeError, VimbaFeatureError):
                        logging.warning("Failed to adjust packet size for camera %s", camera.get_id())

                if self.settings["camera_auto_features"]["auto_exposure"]:
                    try:
                        camera.ExposureAuto.set('Continuous')  # TODO add option to also set 'Once' mode?
                    except (AttributeError, VimbaFeatureError):
                        logging.warning("Failed to set exposure for camera %s", camera.get_id())

                if self.settings["camera_auto_features"]["auto_white_balance"]:
                    try:
                        camera.BalanceWhiteAuto.set('Continuous')  # TODO add option to also set 'Once' mode?
                    except (AttributeError, VimbaFeatureError):
                        logging.warning("Failed to set white balance for camera %s", camera.get_id())

                if self.settings["camera_auto_features"]["auto_iso"]:
                    try:
                        camera.GainAuto.set('Continuous')  # TODO add option to also set 'Once' mode?
                    except (AttributeError, VimbaFeatureError):
                        logging.warning("Failed to set iso for camera %s", camera.get_id())

                try:
                    # TODO make this less hardcoded
                    camera.set_pixel_format(intersect_pixel_formats(camera.get_pixel_formats(), COLOR_PIXEL_FORMATS)[0])
                except (AttributeError, VimbaFeatureError):
                    logging.warning("Failed to set pixel format for camera %s", camera.get_id())

    def _camera_frame_generator_wrapper(self, cam: Camera) -> np.ndarray:

        with Vimba.get_instance():
            with cam:

                if self.settings["enabled_features"]["video_undistort"]:
                    # load calibration data for this camera
                    with open("camera/calib_coefs_%s.pkl" % cam.get_serial(), "rb") as f:
                        calib_data = pickle.load(f)

                # NOTE timeout does not influence FPS, driver will wait max timeout_ms for the next frame, else it will stop
                default_generator = cam.get_frame_generator(limit=None, timeout_ms=5000)

                for frame in default_generator:
                    try:
                        ndarray_frame = frame.as_numpy_ndarray()
                    except:
                        # TODO this is to avoid the weird random error "parameter -7 invalid" which happens rarely
                        continue

                    image = cv2.cvtColor(ndarray_frame, cv2.COLOR_BAYER_RG2RGB)

                    # TODO resize frame here, or later in the app? Advantage is to undistort a smaller frame for better speed?

                    if self.settings["enabled_features"]["video_undistort"]:
                        h, w = image.shape[:2]
                        cameramtx, roi = cv2.getOptimalNewCameraMatrix(
                            calib_data['mtx'],
                            calib_data['dist'],
                            (w, h),
                            1,
                            (w, h)
                        )
                        corrected = cv2.undistort(image, calib_data['mtx'], calib_data['dist'], None, cameramtx)  # FIXME
                        x, y, w, h = roi
                        corrected = corrected[y:y + h, x:x + w]
                        yield corrected
                    else:
                        yield image

    def stream_generator(self) -> Dict[str, object]:
        """Generator that returns timestamped images + sensor data"""

        with Vimba.get_instance():

            while True:

                return_packet = {
                    "images": {},
                    "sensor_data": {},
                    "datetime": datetime.now()
                }

                # TODO not sure if the three cameras will be in sync, buffers might need to be flushed somehow
                if self.settings["enabled_features"]["video"]:
                    for pos, gen in self.frame_generators.items():
                        return_packet["images"][pos] = next(gen)

                # get sensor data

                if self.imu_device is not None:

                    imu_batch = self.imu_device.getStreamingBatch()

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

                gps_msgs = self.gps_provider.get_latest_messages()

                return_packet["sensor_data"]["gps"] = {}

                for msg_type, msg_value in gps_msgs.items():
                    return_packet["sensor_data"]["gps"][msg_type] = str(msg_value)

                yield return_packet
