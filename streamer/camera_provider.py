import time
from multiprocessing import Manager, Lock, Process, Event
from typing import Optional
import cv2
from pymba import Vimba, Frame, camera
import ctypes


class CameraProvider:

    def __init__(self, camera_id):
        self.camera_id = camera_id

        self.last_time = time.time()

        self.PIXEL_FORMATS_CONVERSIONS = {
            'BayerRG8': cv2.COLOR_BAYER_RG2RGB,
            # TODO add more formats
        }

        manager = Manager()  # fun fact, make this self.manager and watch the world burn

        self._frame_cache = manager.Array(ctypes.c_double, )
        self._frame_cache_lock = Lock()

        self._worker_running = manager.Event()
        self._worker_running.set()
        self._worker = Process(
            target=self._blueprint,
            args=(self.camera_id, self._worker_running)
        )
        self._worker.start()

        # DO NOT DEFINE EXTRA ARGUMENTS AFTER WORKER START!

    def _on_frame_ready(self, frame: Frame, delay: Optional[int] = 1) -> None:
        # print('frame {}'.format(frame.data.frameID))

        # get a copy of the frame data
        image = frame.buffer_data_numpy()

        # convert colour space if desired
        try:
            image = cv2.cvtColor(image, self.PIXEL_FORMATS_CONVERSIONS[frame.pixel_format])
        except KeyError:
            pass

        # display image

        fps = 1 / (time.time() - self.last_time)
        self.last_time = time.time()

        frame_small = cv2.resize(image, (int(image.shape[1] / 2), int(image.shape[0] / 2)))

        cv2.putText(frame_small, f"FPS: {fps:.1f}", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow(str(self.camera_id), frame_small)

        # print(str(self.camera_id), " FPS: ", fps)

        cv2.waitKey(delay)

    def _blueprint(self, cam_id,  is_running):

        with Vimba() as vimba:
            camera = vimba.camera(cam_id)
            camera.open(adjust_packet_size=False)  # TODO auto adjust packet size changes StreamBytesPerSecond???? NO it just switches to a different mode

            camera.feature("StreamBytesPerSecond").value = 38000000

            # arm the camera and provide a function to be called upon frame ready
            camera.arm('Continuous', self._on_frame_ready)
            camera.start_frame_acquisition()

            # stream images for a while...
            while is_running.is_set():
                # TODO copy frames
                pass

            # stop frame acquisition
            # start_frame_acquisition can simply be called again if the camera is still armed
            camera.stop_frame_acquisition()
            camera.disarm()

            camera.close()

    def close(self) -> None:
        """
        This method terminates the worker process, clears the cache, and frees all other used resources
        in preparation for a graceful shutdown
        """

        # stop worker process
        self._worker_running.clear()
        self._worker.join()

    def __enter__(self):
        """This allows the CameraProvider to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the CameraProvider to be (optionally) used in python 'with' statements"""
        self.close()


if __name__ == '__main__':

    with Vimba() as vimba:
        print(vimba.camera_ids())

    cp0 = CameraProvider("DEV_000F315CD617")
    cp1 = CameraProvider("DEV_000F315CDD58")
    cp2 = CameraProvider("DEV_000F315CDD5B")

    try:
        # time.sleep(15)
        while True:
            pass
    finally:
        cp0.close()
        cp1.close()
        cp2.close()
