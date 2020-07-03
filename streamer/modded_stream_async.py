import threading
import sys
import cv2
import time
import yaml
import logging
from typing import Optional
from typing import Dict
from vimba import *


def abort(reason: str, return_code: int = 1, usage: bool = False):
    print(reason + '\n')
    sys.exit(return_code)


# def get_camera(camera_id: Optional[str]) -> Camera:
#     with Vimba.get_instance() as vimba:
#         if camera_id:
#             try:
#                 return vimba.get_camera_by_id(camera_id)
#
#             except VimbaCameraError:
#                 abort('Failed to access Camera \'{}\'. Abort.'.format(camera_id))
#
#         else:
#             cams = vimba.get_all_cameras()
#             if not cams:
#                 abort('No Cameras accessible. Abort.')
#
#             return cams[0]
#
#
def setup_camera(cam: Camera):
    with cam:
        # Enable auto exposure time setting if camera supports it
        try:
            cam.ExposureAuto.set('Continuous')

        except (AttributeError, VimbaFeatureError):
            pass

        # Enable white balancing if camera supports it
        try:
            cam.BalanceWhiteAuto.set('Continuous')

        except (AttributeError, VimbaFeatureError):
            pass

        # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
        try:
            cam.GVSPAdjustPacketSize.run()

            while not cam.GVSPAdjustPacketSize.is_done():
                pass

        except (AttributeError, VimbaFeatureError):
            pass

        # Query available, open_cv compatible pixel formats
        # prefer color formats over monochrome formats

    cam.set_pixel_format(intersect_pixel_formats(cam.get_pixel_formats(), COLOR_PIXEL_FORMATS)[0])


class MulticamStreamer:

    def __init__(self):
        # load settings from configuration file
        with open("config.yaml", "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        if self.settings["enabled_features"]["video"]:
            # assign cameras to their positions and check everything is working
            self.cameras = self._get_cameras()

            self.handlers = {}

            # setup the cameras by sending vimba commands
            for pos, cam in self.cameras.items():
                self._setup_camera(cam)
                self.handlers[pos] = Handler()

        #     with Vimba.get_instance():
            #         with cam:
            #             cam.start_streaming(handler=self.handlers[pos], buffer_count=10) # TODO adjust buffer

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



class Handler:
    def __init__(self):
        self.shutdown_event = threading.Event()
        self.last_time = time.time()

    def __call__(self, cam: Camera, frame: Frame):
        ENTER_KEY_CODE = 13

        key = cv2.waitKey(1)
        if key == ENTER_KEY_CODE:
            self.shutdown_event.set()
            return

        elif frame.get_status() == FrameStatus.Complete:  # TODO this FrameStatus.Complete might be important
            print('{} acquired {}'.format(cam, frame), flush=True)

            msg = 'Stream from \'{}\'. Press <Enter> to stop stream.'
            # cv2.imshow(msg.format(cam.get_name()), frame.as_opencv_image())

            ndarray_frame = frame.as_numpy_ndarray()

            color_frame = cv2.cvtColor(ndarray_frame, cv2.COLOR_BAYER_RG2RGB)

            fps = 1 / (time.time() - self.last_time)

            cv2.putText(color_frame, f"FPS: {fps:.1f}", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

            cv2.imshow(msg.format(cam.get_name()), color_frame)

            print("FPS: ", fps)
            self.last_time = time.time()

        cam.queue_frame(frame)


def main():

    ms = MulticamStreamer()


    with Vimba.get_instance():
        with ms.cameras["center"] as cam:

            # Start Streaming, wait for five seconds, stop streaming
            #ms._setup_camera(cam)  #setup_camera(cam)
            handler = ms.handlers["center"] #Handler()

            try:
                # Start Streaming with a custom a buffer of 10 Frames (defaults to 5)
                cam.start_streaming(handler=handler, buffer_count=25)

                # while True:
                #     print(handler.last_time)
                #
                # handler.shutdown_event.wait()
            finally:
                cam.stop_streaming()


if __name__ == '__main__':
    main()
