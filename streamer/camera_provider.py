import time
import logging
import functools
from copy import deepcopy
from multiprocessing import Manager, Lock, Process, Event, Value, shared_memory
from ctypes import c_char_p
from typing import Optional, Dict

import numpy as np
import pickle
import cv2
from pymba import Vimba, Frame, camera

# todo add more colours
PIXEL_FORMATS_CONVERSIONS = {
    'BayerRG8': cv2.COLOR_BAYER_RG2RGB,
}


class CameraSharedMemProvider:

    def __init__(self, camera_id):
        """
        CameraSharedMemProvider spawns a child worker process which constantly receives data from the camera.
        When getter methods are called, the shared memory cache of the worker is interrogated to get the latest
        available frame asynchronously
        """

        self.camera_id = camera_id

        manager = Manager()  # fun fact, make this self.manager and watch the world burn
        self._worker_running = manager.Event()
        self._worker_initialized = manager.Event()
        self._shm_addr = manager.Value(c_char_p, "")
        self._buffer_dtype = manager.Value(c_char_p, "")
        self._buffer_shape = manager.Value(c_char_p, "")
        self._pixel_format = manager.Value(c_char_p, "")
        self._frame_buffer_lock = Lock()

        self._worker = None
        self._spawn_worker()

        # DO NOT DEFINE EXTRA ARGUMENTS AFTER WORKER START!

    def _spawn_worker(self) -> None:

        logging.debug(f"CameraProvider {self.camera_id} is spawning a new worker process")

        self._worker_running.set()
        self._worker = Process(
                          target=self._blueprint,
                          args=(
                              self.camera_id,
                              self._shm_addr,
                              self._buffer_dtype,
                              self._buffer_shape,
                              self._pixel_format,
                              self._frame_buffer_lock,
                              self._worker_running,
                              self._worker_initialized
                          )
                      )
        self._worker.start()
        self._worker_initialized.wait()  # wait for the shared memory block address
        self._child_shm = shared_memory.SharedMemory(name=self._shm_addr.value)

        _shape = pickle.loads(self._buffer_shape.value)
        _dtype = pickle.loads(self._buffer_dtype.value)
        self._child_frame_buffer = np.ndarray(shape=_shape, dtype=_dtype, buffer=self._child_shm.buf)

        logging.debug(f"CameraProvider {self.camera_id} process init successful")

    def _blueprint(self, camera_id: str, shm_addr: Value, buffer_dtype: Value, buffer_shape: Value,
                   pixel_format: Value, frame_buffer_lock: Lock, is_running: Event, worker_initialized: Event) -> None:

        with Vimba() as vimba:
            camera = vimba.camera(camera_id)
            camera.open(adjust_packet_size=False)  # TODO auto adjust packet size changes StreamBytesPerSecond???? NO it just switches to a different mode
            camera.feature("StreamBytesPerSecond").value = 38000000

            shm_primitives = {
                "shm": None,  # will be initialized by the vimba callback
                "frame_buffer": None,  # will be initialized by the vimba callback
                "frame_buffer_lock": frame_buffer_lock,
                "ready_event": worker_initialized,
                "shm_addr": shm_addr,
                "buffer_dtype": buffer_dtype,
                "buffer_shape": buffer_shape,
                "pixel_format": pixel_format
            }

            _callback = functools.partial(self._on_frame_ready, shared_mem_primitives=shm_primitives)

            camera.arm('Continuous', _callback)
            camera.start_frame_acquisition()

            while is_running.is_set():
                # TODO monitor state and attempt to restart worker if camera times out
                time.sleep(1)

            camera.stop_frame_acquisition()
            camera.disarm()

            camera.close()

            worker_initialized.clear()
            del shm_primitives["frame_buffer"]
            shm_primitives["shm"].close()

            logging.debug("Provider worker process has stopped correctly")  # TODO enable logging from worker process

    def _on_frame_ready(self, frame: Frame, shared_mem_primitives: Dict, delay: Optional[int] = 1) -> None:

        if not shared_mem_primitives["ready_event"].is_set():
            # init shared memory on first frame captured
            first_frame = frame.buffer_data_numpy()

            shared_mem_primitives["shm"] = shared_memory.SharedMemory(create=True, size=first_frame.nbytes)
            shared_mem_primitives["frame_buffer"] = np.ndarray(first_frame.shape, dtype=first_frame.dtype, buffer=shared_mem_primitives["shm"].buf)

            shared_mem_primitives["frame_buffer"] = first_frame[:]

            shared_mem_primitives["shm_addr"].value = shared_mem_primitives["shm"].name
            shared_mem_primitives["buffer_dtype"].value = pickle.dumps(first_frame.dtype)
            shared_mem_primitives["buffer_shape"].value = pickle.dumps(first_frame.shape)
            shared_mem_primitives["pixel_fomat"].value = frame.pixel_format

            # notify main process that shared memory is ready to use
            shared_mem_primitives["ready_event"].set()

        else:
            shared_mem_primitives["frame_buffer_lock"].acquire()
            shared_mem_primitives["frame_buffer"][:] = frame.buffer_data_numpy()[:]
            shared_mem_primitives["frame_buffer_lock"].release()

    def close(self) -> None:
        """
        This method terminates the worker process, clears the buffer, and frees all other used resources
        in preparation for a graceful shutdown
        """

        # stop worker process
        self._worker_running.clear()
        self._worker.join()

        # clear buffers just to be sure
        self._frame_buffer_lock.acquire()

        del self._child_frame_buffer
        self._child_shm.close()
        self._child_shm.unlink()

        self._frame_buffer_lock.release()

        logging.debug("Provider has cleaned up used resources")

    def __enter__(self):
        """This allows the provider to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the provider to be (optionally) used in python 'with' statements"""
        self.close()

    def get_last_frame_raw(self) -> np.ndarray:
        """Get the latest frame"""

        # TODO check worker.is_alive() and check timestamps for the packages are reasonable,
        #  else clear the buffer and attempt to restart the worker?

        self._frame_buffer_lock.acquire()

        buffer_copy = deepcopy(self._child_frame_buffer)

        self._frame_buffer_lock.release()

        return buffer_copy

    def get_last_frame_as_ocv(self) -> np.ndarray:
        """Get the latest frame in opencv compatible format"""
        raw_frame = self.get_last_frame_raw()
        return cv2.cvtColor(raw_frame, PIXEL_FORMATS_CONVERSIONS[self._pixel_format.value])


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)

    last_time = time.time()

    with CameraSharedMemProvider("DEV_000F315CD617") as p:
        for i in range(1000):

            image = p.get_last_frame_as_ocv()

            fps = 1 / (time.time() - last_time)
            last_time = time.time()

            frame_small = cv2.resize(image, (int(image.shape[1] / 2), int(image.shape[0] / 2)))

            cv2.putText(frame_small, f"FPS: {fps:.1f}", (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

            cv2.imshow("stream", frame_small)
            cv2.waitKey(1)
