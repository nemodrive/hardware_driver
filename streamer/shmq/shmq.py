from multiprocessing import Process, Manager, Queue, Event, Lock, shared_memory, Value
from typing import Tuple, Union
from ctypes import c_int64, c_bool
import time
import numpy as np

import time


class SharedMemoryQueue:

    def __init__(self, element_shape: Tuple[int, ...], max_size: int, dtype: object):
        self._element_shape = element_shape
        self._max_size = max_size
        self._dtype = dtype

        # init synchronization mechanism

        manager = Manager()

        self._written_indices = manager.Queue(maxsize=self._max_size)

        self._write_available = manager.Event()
        self._buffer_status_lock = Lock()
        self._buffer_status_lock.acquire()

        # block all operations until initialized

        self._write_available.clear()

        # create the buffer

        self._buffer_locks = [Lock() for _ in range(max_size)]
        self._buffer_arrays = []
        self._buffer_shms = []
        self._buffer_shms_names = []  # no need to use Value, they will be created in main thread and used as read only
        self._buffer_free = [manager.Value(c_bool, True) for _ in range(max_size)]  # protected by buffer status lock
        self._crt_buffer_size = manager.Value(c_int64, 0)  # protected by buffer status lock

        self._nbytes_per_elem = np.empty(self._element_shape, dtype=self._dtype).nbytes

        for i in range(self._max_size):

            self._buffer_shms.append(shared_memory.SharedMemory(create=True, size=self._nbytes_per_elem))
            self._buffer_shms_names.append(self._buffer_shms[i].name)
            self._buffer_arrays.append(np.ndarray(self._element_shape, dtype=self._dtype,
                                                  buffer=self._buffer_shms[i].buf))

        self._write_available.set()
        self._buffer_status_lock.release()

    @property
    def element_shape(self):
        return self._element_shape

    @property
    def max_size(self):
        return self._max_size

    @property
    def dtype(self):
        return self._dtype

    def register_worker(self):
        # link local memory to the shared blocks

        for i in range(self._max_size):

            self._buffer_shms[i] = shared_memory.SharedMemory(name=self._buffer_shms_names[i])
            self._buffer_arrays[i] = np.ndarray(self._element_shape, dtype=self._dtype, buffer=self._buffer_shms[i].buf)

    def put(self, array: np.ndarray):

        while True:

            self._buffer_status_lock.acquire()

            if self._write_available.is_set():
                break

            self._buffer_status_lock.release()

        # search for a free position

        found_index = None

        for index, is_free_mv in enumerate(self._buffer_free):

            if is_free_mv.value:
                found_index = index
                self._buffer_free[found_index].value = False
                self._buffer_locks[found_index].acquire()
                break

        self._crt_buffer_size.value = self._crt_buffer_size.value + 1

        if self._crt_buffer_size.value == self._max_size:
            self._write_available.clear()

        self._buffer_status_lock.release()

        if found_index is None:
            raise Exception("ERROR WRITE READY EVENT SET WHEN IT SHOULD NOT BE!!!!")

        # write to free position
        self._buffer_arrays[found_index][:] = array[:]
        self._written_indices.put(found_index)
        self._buffer_locks[found_index].release()

    def send_close_signal(self):
        self._written_indices.put(None)

    def get(self) -> Union[np.ndarray, None]:

        crt_element_index = self._written_indices.get()

        if crt_element_index is None:
            return None

        # read the value

        self._buffer_locks[crt_element_index].acquire()

        return_value = np.ndarray(shape=self._element_shape, dtype=self._dtype, buffer=self._buffer_shms[crt_element_index].buf).copy()

        self._buffer_locks[crt_element_index].release()

        # announce position free

        self._buffer_status_lock.acquire()

        self._buffer_free[crt_element_index].value = True
        self._crt_buffer_size.value = self._crt_buffer_size.value - 1

        if not self._write_available.is_set():
            self._write_available.set()

        self._buffer_status_lock.release()

        return return_value

    def free(self):
        self._buffer_status_lock.acquire()

        for i in range(self._max_size):

            self._buffer_locks[i].acquire()

            self._buffer_shms[i].close()
            self._buffer_shms[i].unlink()


def producer(queue: SharedMemoryQueue):

    queue.register_worker()

    time.sleep(2)

    print("producer start!")

    for i in range(60):

        crt_time = time.time()

        #tmp_arr = np.zeros((2500, 2500, 3), dtype=np.int64)
        tmp_arr = np.random.rand(3, 2560, 1440)  # float64
        tmp_arr[0, 0, 0] = i

        queue.put(tmp_arr)

        print(f"----> I put {tmp_arr[0, 0, 0]} in {time.time() - crt_time}")

        time.sleep(0.1)


def consumer(queue: SharedMemoryQueue):

    print("consumer start")

    queue.register_worker()

    job_start_time = time.time()

    for i in range(60):

        crt_time = time.time()

        arr = queue.get_via_cache()

        print(f"<---- Received arr {arr[0, 0, 0]} in {time.time() - crt_time}")

        time.sleep(0.5)

    print(f"Recv all after {time.time() - job_start_time} !!!")


if __name__ == '__main__':

    shm_queue = SharedMemoryQueue((3, 2560, 1440), 60, np.float64)

    # shm_queue = Manager().Queue(maxsize=5)

    prod_proc = Process(target=producer, args=(shm_queue, ))
    cons_proc = Process(target=consumer, args=(shm_queue, ))

    prod_proc.start()
    cons_proc.start()

    cons_proc.join()
