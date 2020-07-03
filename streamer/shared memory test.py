import time
import logging
from multiprocessing import Manager, Lock, Process, Event, Value, shared_memory
from ctypes import c_char_p
import numpy as np

from copy import deepcopy


class CameraSharedMemProvider:

    def __init__(self):
        """
        CameraSharedMemProvider spawns a child worker process which constantly receives data from the camera.
        When getter methods are called, the shared memory cache of the worker is interrogated to get the latest
        available frame asynchronously
        """

        manager = Manager()  # fun fact, make this self.manager and watch the world burn
        self._worker_running = manager.Event()
        self._worker_initialized = manager.Event()
        self._frame_cache_lock = Lock()

        self._shm_addr = manager.Value(c_char_p, "")

        self._worker = None
        self._spawn_worker()

    def _spawn_worker(self) -> None:

        logging.debug("Provider is spawning a new worker process")

        self._worker_running.set()
        self._worker = Process(
                          target=self._blueprint,
                          args=(self._shm_addr, self._frame_cache_lock, self._worker_running, self._worker_initialized)
                      )
        self._worker.start()
        self._worker_initialized.wait()  # wait for the shared memory block address
        self._child_shm = shared_memory.SharedMemory(name=self._shm_addr.value)
        self._child_frame_buffer = np.frombuffer(self._child_shm.buf)

        logging.debug("Provider process init successful")

    def _blueprint(self, shm_addr: Value, frame_cache_lock: Lock, is_running: Event, worker_initialized: Event) -> None:

        # TODO get the first frame from the camera
        frame_1 = np.random.random((2064, 1544))

        shm = shared_memory.SharedMemory(create=True, size=frame_1.nbytes)
        frame_buffer = np.ndarray(frame_1.shape, dtype=frame_1.dtype, buffer=shm.buf)

        shm_addr.value = shm.name

        worker_initialized.set()

        while is_running.is_set():

            time.sleep(0.03)

            frame_cache_lock.acquire()

            # TODO get subequent frames from the camera
            frame_buffer[0, 0] = frame_buffer[0, 0] + 1

            frame_cache_lock.release()

        worker_initialized.clear()
        del frame_buffer
        shm.close()

        logging.debug("Provider worker process has stopped correctly")  # TODO enable logging from worker processes

    def close(self) -> None:
        """
        This method terminates the worker process, clears the cache, and frees all other used resources
        in preparation for a graceful shutdown
        """

        # stop worker process
        self._worker_running.clear()
        self._worker.join()

        # clear cache just to be sure
        self._frame_cache_lock.acquire()

        del self._child_frame_buffer
        self._child_shm.close()
        self._child_shm.unlink()

        self._frame_cache_lock.release()

        logging.debug("Provider has cleaned up used resources")

    def __enter__(self):
        """This allows the provider to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the provider to be (optionally) used in python 'with' statements"""
        self.close()

    def get_last_frame(self) -> np.ndarray:
        """Get the latest messages received for each category"""

        # TODO check worker.is_alive() and check timestamps for the packages are reasonable,
        #  else clear the cache and attempt to restart the worker?

        self._frame_cache_lock.acquire()

        cache_copy = deepcopy(self._child_frame_buffer)

        self._frame_cache_lock.release()

        return cache_copy


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)

    with CameraSharedMemProvider() as p:
        for i in range(10):
            crt_cache = p.get_last_frame()

            print(crt_cache)

            # time.sleep(2)

    print("Graceful exit?")
    print(not p._worker.is_alive())
