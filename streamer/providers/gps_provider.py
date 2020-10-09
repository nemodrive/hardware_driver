from typing import Dict
import time
import logging
from multiprocessing import Manager, Lock, Process, Event
from copy import deepcopy

import serial
import pynmea2

NMEACache = Dict[str, pynmea2.TalkerSentence]


class GPSProvider:
    """
    GPSProvider spawns a child worker process which constantly receives data from the gps.
    When getter methods are called, the shared memory cache of the worker is interrogated to get the latest
        messages asynchronously.
    """

    def __init__(self, gps_port: str):
        """
        Instantiates the shared memory buffer and starts the worker.

        Args:
            gps_port (str): Serial port where the gps device is connected
        """

        manager = Manager()  # fun fact, make this self.manager and watch the world burn

        self._gps_cache = manager.dict()
        self._worker_running = manager.Event()
        self._gps_cache_lock = Lock()

        self._data_available = manager.Event()
        self._data_available.clear()

        self._gps_port = gps_port

        self._worker = None
        self._spawn_worker()

    def _spawn_worker(self) -> None:
        """Starts the worker process"""

        logging.debug("GPS Provider is spawning a new worker process")

        self._worker_running.set()
        self._worker = Process(
                          target=self._blueprint,
                          args=(self._gps_port, self._gps_cache, self._gps_cache_lock,
                                self._data_available, self._worker_running)
                      )
        self._worker.start()

    def _blueprint(self, gps_port: str, gps_cache: NMEACache, gps_cache_lock: Lock, data_available: Event,
                   is_running: Event) -> None:
        """
        Blueprint for the GPS worker which will perpetually receive packets from the physical device.

        Args:
            gps_port (str): Serial port where the gps device is connected
            gps_cache (NMEACache): Cache where received packages will be stored
            gps_cache_lock (Lock): Lock for safely accessing the gps_cache
            data_available (Event): Is set if new data was read from the device but not received by the master thread yet
            is_running (Event): The process will shut down when this event is cleared
        """

        with serial.Serial(gps_port, baudrate=9600, timeout=None) as serial_port:

            # FIXME set the correct port or even search for it based on messages received and status
            # TODO auto flushed by receiving \r\n on bus, check the gps module sends this

            # TODO add a buffer? sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser)) in example but
            #  https://stackoverflow.com/questions/24498048/python-io-modules-textiowrapper-or-buffererwpair-functions-are-not-playing-nice

            while is_running.is_set():

                raw_message = serial_port.readline()

                try:
                    nmea_msg = pynmea2.parse(raw_message.decode())
                except pynmea2.nmea.ParseError as e:
                    # package is compromised, maybe checksum verification failed, we'll skip this one
                    logging.debug("GPS received malformed NMEA message!!!")
                    continue

                # TODO also log raw gps output to make sure its ok

                gps_cache_lock.acquire()

                gps_cache[nmea_msg.sentence_type] = nmea_msg

                data_available.set()

                gps_cache_lock.release()

        logging.debug("GPS Provider worker process has stopped correctly")  # TODO enable logging from worker processes

    def close(self) -> None:
        """
        This method terminates the worker process, clears the cache, and frees all other used resources
        in preparation for a graceful shutdown.
        """

        # stop worker process
        self._worker_running.clear()
        self._worker.join()

        # clear cache just to be sure
        self._gps_cache_lock.acquire()
        self._gps_cache.clear()
        self._gps_cache_lock.release()

        logging.debug("GPS Provider has cleaned up used resources")

    def __enter__(self):
        """This allows the GPSProvider to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the GPSProvider to be (optionally) used in python 'with' statements"""
        self.close()

    def get_latest_messages(self) -> NMEACache:
        """
        Get the latest messages received for each category.

        Returns:
            NMEACache: A compilation of the last known value for each message type
        """

        # TODO check worker.is_alive() and check timestamps for the packages are reasonable,
        #  else clear the cache and attempt to restart the worker?

        self._gps_cache_lock.acquire()

        cache_copy = deepcopy(self._gps_cache)

        self._gps_cache.clear()
        self._data_available.clear()

        self._gps_cache_lock.release()

        return cache_copy

    def has_unread_data(self):
        return self._data_available.is_set()


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)

    with GPSProvider('/dev/ttyACM0') as p:
        for i in range(1000):

            if p.has_unread_data():
                print(p.get_latest_messages())
            else:
                print(None)

            # time.sleep(2)

    print("Graceful exit?")
    print(not p._worker.is_alive())
