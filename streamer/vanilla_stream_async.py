"""BSD 2-Clause License

Copyright (c) 2019, Allied Vision Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE SOFTWARE IS PRELIMINARY AND STILL IN TESTING AND VERIFICATION PHASE AND
IS PROVIDED ON AN “AS IS” AND “AS AVAILABLE” BASIS AND IS BELIEVED TO CONTAIN DEFECTS.
A PRIMARY PURPOSE OF THIS EARLY ACCESS IS TO OBTAIN FEEDBACK ON PERFORMANCE AND
THE IDENTIFICATION OF DEFECT SOFTWARE, HARDWARE AND DOCUMENTATION.
"""
import threading
import sys
import cv2
import time
from typing import Optional
from vimba import *


def print_preamble():
    print('///////////////////////////////////////////////////////')
    print('/// Vimba API Asynchronous Grab with OpenCV Example ///')
    print('///////////////////////////////////////////////////////\n')


def print_usage():
    print('Usage:')
    print('    python asynchronous_grab_opencv.py [camera_id]')
    print('    python asynchronous_grab_opencv.py [/h] [-h]')
    print()
    print('Parameters:')
    print('    camera_id   ID of the camera to use (using first camera if not specified)')
    print()


def abort(reason: str, return_code: int = 1, usage: bool = False):
    print(reason + '\n')

    if usage:
        print_usage()

    sys.exit(return_code)


def parse_args() -> Optional[str]:
    args = sys.argv[1:]
    argc = len(args)

    for arg in args:
        if arg in ('/h', '-h'):
            print_usage()
            sys.exit(0)

    if argc > 1:
        abort(reason="Invalid number of arguments. Abort.", return_code=2, usage=True)

    return None if argc == 0 else args[0]


def get_camera(camera_id: Optional[str]) -> Camera:
    with Vimba.get_instance() as vimba:
        if camera_id:
            try:
                return vimba.get_camera_by_id(camera_id)

            except VimbaCameraError:
                abort('Failed to access Camera \'{}\'. Abort.'.format(camera_id))

        else:
            cams = vimba.get_all_cameras()
            if not cams:
                abort('No Cameras accessible. Abort.')

            return cams[0]


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
        cam.GVSPPacketSize.set(500)
        # try:
        #     cam.GVSPAdjustPacketSize.run()
        #
        #     while not cam.GVSPAdjustPacketSize.is_done():
        #         pass
        #
        # except (AttributeError, VimbaFeatureError):
        #     pass

        # Query available, open_cv compatible pixel formats
        # prefer color formats over monochrome formats
        
    cam.set_pixel_format(intersect_pixel_formats(cam.get_pixel_formats(), COLOR_PIXEL_FORMATS)[0])


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

            cv2.imshow(cam.get_id(), color_frame)

            print("FPS: ", fps)
            self.last_time = time.time()

        cam.queue_frame(frame)


def main():
    print_preamble()
    cam_id = parse_args()

    with Vimba.get_instance():
        with get_camera(cam_id) as cam:

            # Start Streaming, wait for five seconds, stop streaming
            setup_camera(cam)
            handler = Handler()

            try:
                # Start Streaming with a custom a buffer of 10 Frames (defaults to 5)
                cam.start_streaming(handler=handler, buffer_count=25)

                while True:
                    print(handler.last_time)

                handler.shutdown_event.wait()
            finally:
                cam.stop_streaming()


if __name__ == '__main__':
    main()
