import cantools
import can

from typing import Dict, List, Any
import time
import logging
from multiprocessing import Manager, Lock, Process, Event
import queue
from copy import deepcopy


class CanbusProvider:
    def __init__(self, can_device: str = "can0", dbc_file: str = "logan.dbc"):
        """
            CanbusProvider works in a manner similar to GPS Provider.
            It spawns a child worker process which constantly receives data from the CAN device.
            When getter methods are called, the shared memory cache of the worker is interrogated to get the latest
            messages asynchronously.

            :param can_device: name of the device
            :param dbc_file: name of the DBC file used to decode CAN messages related to speed
        """
        self._can_device = can_device

        manager = Manager()  # fun fact, make this self.manager and watch the world burn

        self._canbus_cache = manager.dict()
        self._worker_running = manager.Event()
        self._canbus_cache_lock = Lock()

        self._can_dbc = cantools.db.load_file(dbc_file, strict=False)

        self._worker = None
        self._spawn_worker()

        self.current_speed = 0.0

    def _spawn_worker(self) -> None:
        logging.debug("CanbusProvider is spawning a new worker process")

        self._worker_running.set()
        self._worker = Process(
            target=self._can_communicator,
            args=(self._can_device, self._can_dbc, self._canbus_cache, self._canbus_cache_lock, self._worker_running)
        )
        self._worker.start()

    def _can_communicator(self, can_device: str, can_dbc: Any,
                          canbus_cache: Dict[str, Any],
                          canbus_cache_lock: Lock, is_running: Event) -> None:
        CAN_CMD_NAMES = dict({
            # cmd_name, data_name, can_id
            "speed": ("SPEED_SENSOR", "SPEED_KPS", "354"),
            "steer": ("STEERING_SENSORS", "STEER_ANGLE", "0C6"),
            # TODO what is the id? "brake": ("BRAKE_SENSOR", "PRESIUNE_C_P")
        })

        with can.interface.Bus(can_device, bustype='socketcan') as can_bus:
            while is_running.is_set():
                msg = can_bus.recv()

                logging.debug("Message from CAN %s" % msg)

                for cmd_key, cmd_can in CAN_CMD_NAMES:
                    cmd_name, data_name, can_id_str = cmd_can

                    # verify the type of the message by means of the can_id
                    if msg.arbitration_id == int(can_id_str, 16):
                        msg_data = can_dbc.decode_message(msg.arbitration_id, msg.data)
                        value = msg_data[data_name]  # TODO conversions for all

                        # acquire lock and insert data
                        canbus_cache_lock.acquire()

                        canbus_cache[cmd_key] = {
                            "value": value,
                            "timestamp": msg.timestamp
                        }

                        canbus_cache_lock.release()

        logging.debug("Speed Provider worker process has stopped correctly")

    def close(self) -> None:
        """
        This method terminates the worker process, clears the cache, and frees all other used resources
        in preparation for a graceful shutdown
        """

        # stop worker process
        self._worker_running.clear()
        self._worker.join()

        # clear cache just to be sure
        self._canbus_cache_lock.acquire()

        for m in self._canbus_cache.keys():
            self._canbus_cache[m] = None

        self._canbus_cache_lock.release()

        logging.debug("CanbusProvider has cleaned up used resources")

    def __enter__(self):
        """This allows the CanbusProvider to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the CanbusProvider to be (optionally) used in python 'with' statements"""
        self.close()

    def get_latest_messages(self) -> Dict[str, Any]:
        """Get the latest speed readings since last call"""

        # TODO check worker.is_alive() and check timestamps for the packages are reasonable,
        #  else clear the cache and attempt to restart the worker?
        # speed_readings: List[SpeedData] = []

        self._canbus_cache_lock.acquire()

        cache_copy = deepcopy(self._canbus_cache)

        self._canbus_cache_lock.release()

        return cache_copy


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    """
    bring up can: https://elinux.org/Bringing_CAN_interface_up
    """

    with CanbusProvider(can_device='slcan0', dbc_file="../logan.dbc") as p:
        for i in range(10):
            crt_cache = p.get_latest_messages()

            print(crt_cache)

            time.sleep(2)

    print("Graceful exit?")
    print(not p._worker.is_alive())
