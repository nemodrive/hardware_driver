import yaml
import numpy as np
import cv2
import pickle
import logging
import time
from datetime import datetime
from typing import Dict

import libs.yei3.threespace_api as ts_api
from gps_provider import GPSProvider

from vimba import *


logging.basicConfig(level="DEBUG")


class MulticamStreamer:

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

                # camera.StreamBytesPerSecond.set(38000000)

                if self.settings["camera_auto_features"]["auto_adjust_packet_size"]:
                    #camera.GVSPPacketSize.set(1500)
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
                default_generator = cam.get_frame_generator(limit=None, timeout_ms=500000)

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


if __name__ == '__main__':

    ms = MulticamStreamer()

    with Vimba.get_instance():
        
        per_camera_frametimes = {pos: time.time() for pos in ms.frame_generators.keys()}

        while True:
            for pos, gen in ms.frame_generators.items():
                frame = next(gen)
                fps = 1/(time.time() - per_camera_frametimes[pos])

                per_camera_frametimes[pos] = time.time()

                frame_small = cv2.resize(frame, (int(frame.shape[1] / 2), int(frame.shape[0] / 2)))
                
                cv2.putText(frame_small, f"FPS: {fps:.1f}", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                cv2.imshow(pos, frame_small)
                cv2.waitKey(1)
