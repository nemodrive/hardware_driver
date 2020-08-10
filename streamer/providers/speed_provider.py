import cantools
import can

from typing import Dict, List, Any
import time
import logging
from multiprocessing import Manager, Lock, Process, Event
import queue

class SpeedData:
    def __init__(self, timestamp: float, speed: float) -> None:
        """
        Records a speed measurement taken from the CAN of the car
        :param timestamp: float value representing the timestamp of the CAN message from which the speed value
        was extracted
        :param speed: wheel speed of the vehicle in meters per second
        """
        self.timestamp = timestamp
        self.speed = speed

    def __str__(self):
        return "SpeedData[ts=%.4f, speed=%6.2f]" % (self.timestamp, self.speed)


class SpeedProvider:
    def __init__(self, can_device:str="can0", dbc_file:str="logan.dbc"):
        """
            SpeedProvider works in a manner similar to GPS Provider.
            It spawns a child worker process which constantly receives data from the CAN device.
            When getter methods are called, the shared memory cache of the worker is interrogated to get the latest
            messages asynchronously.

            :param can_device: name of the device
            :param dbc_file: name of the DBC file used to decode CAN messages related to speed
        """
        self._can_device = can_device

        manager = Manager()  # fun fact, make this self.manager and watch the world burn

        self._speed_cache = manager.Queue()
        self._worker_running = manager.Event()
        self._speed_cache_lock = Lock()

        self._can_dbc = cantools.db.load_file(dbc_file, strict=False)

        self._worker = None
        self._spawn_worker()

    def _spawn_worker(self) -> None:
        logging.debug("Speed Provider is spawning a new worker process")

        self._worker_running.set()
        self._worker = Process(
            target=self._can_communicator,
            args=(self._can_device, self._can_dbc, self._speed_cache, self._speed_cache_lock, self._worker_running)
        )
        self._worker.start()

    def _can_communicator(self, can_device: str, can_dbc: Any,
                          speed_cache: queue.Queue,
                          speed_cache_lock: Lock, is_running: Event) -> None:
        CAN_CMD_NAMES = dict({
            # cmd_name, data_name, can_id
            "speed": ("SPEED_SENSOR", "SPEED_KPS", "354"),
            "steer": ("STEERING_SENSORS", "STEER_ANGLE", "0C6"),
            "brake": ("BRAKE_SENSOR", "PRESIUNE_C_P")
        })

        with can.interface.Bus(can_device, bustype='socketcan') as can_bus:
            while is_running.is_set():
                msg = can_bus.recv()

                logging.debug("Message from CAN %s" % msg)

                # verify if the message is a speed reading by means of the can_id
                cmd_name, data_name, can_id_str = CAN_CMD_NAMES["speed"]
                if msg.arbitration_id == int(can_id_str, 16):
                    msg_data = can_dbc.decode_message(msg.arbitration_id, msg.data)
                    speed = msg_data[data_name]  # float number of speed in km/h
                    speed_mps = speed / 3.6

                    speed_data = SpeedData(msg.timestamp, speed_mps)


                    # acquire lock and insert speed
                    speed_cache_lock.acquire()
                    speed_cache.put(speed_data)
                    speed_cache_lock.release()

        logging.debug("Speed Provider worker process has stopped correctly")

    def close(self) -> None:
        """
        This method terminates the worker process, clears the cache, and frees all other used resources
        in preparation for a graceful shutdown
        """

        # stop worker process
        self._worker_running.clear()
        self._worker.join()

        # clear cache queue just to be sure
        self._speed_cache_lock.acquire()

        while not self._speed_cache.empty():
            self._speed_cache.get_nowait()

        self._speed_cache_lock.release()
        logging.debug("Speed Provider has cleaned up used resources")

    def __enter__(self):
        """This allows the SpeedProvider to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the SpeedProvider to be (optionally) used in python 'with' statements"""
        self.close()

    def get_latest_messages(self) -> List[SpeedData]:
        """Get the latest speed readings since last call"""

        # TODO check worker.is_alive() and check timestamps for the packages are reasonable,
        #  else clear the cache and attempt to restart the worker?
        speed_readings: List[SpeedData] = []

        self._speed_cache_lock.acquire()

        # get all the messages from the Queue until it is empty
        while not self._speed_cache.empty():
            speed_data = self._speed_cache.get_nowait()
            speed_readings.append(speed_data)
            self._speed_cache.task_done()

        self._speed_cache_lock.release()

        return speed_readings


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    with SpeedProvider(can_device='vcan0', dbc_file="logan.dbc") as p:
        for i in range(10):
            crt_cache = p.get_latest_messages()
            for speed_data in crt_cache:
                print("\tLatest Speed Message:", speed_data)

            time.sleep(2)

    print("Graceful exit?")
    print(not p._worker.is_alive())
