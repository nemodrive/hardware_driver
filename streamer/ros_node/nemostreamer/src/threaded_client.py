import socket
import logging
import time
import zlib
from Queue import Empty
from multiprocessing import Process, Manager, Event, Queue

import pickle


class ThreadedBroadcastClient:
    """
    Receives Python objects from a ThreadedBroadcastServer.
    Uses a Queue and a separate process to receive packets asynchronously from the server.
    The user can then pop items from this queue using the recv_object*() methods.
    """

    def __init__(self, host, port, header_size, buffer_size):
        """
        Instantiates the client and starts the worker process which immediately begins to receive packets.

        Args:
            host (str): Server host address
            port (int): Server port where new connection requests will be sent
            header_size (int): Number of bytes used to represent object size
            buffer_size (int): Size for the buffer used by the receiving socket
        """

        self.host = host
        self.port = port
        self.header_size = header_size
        self.buffer_size = buffer_size

        manager = Manager()

        self._packets_queue = manager.Queue()
        self._worker_running = manager.Event()
        self._worker_running.set()

        self._worker = Process(target=self._receive_packets_thread, args=(self._packets_queue, self._worker_running))
        self._worker.start()

    def _receive_packets_thread(self, output_queue, is_running):
        """
        Blueprint method for the dedicated thread which perpetually receives packets from the server.

        Args:
            output_queue (Queue): Queue where received objects will be added
            is_running (Event): The process will shut down when this event is cleared
        """

        client_socket = socket.socket()

        try:
            client_socket.connect((self.host, self.port))
        except socket.error as e:
            logging.error(e)

        while is_running.is_set():

            try:
                _begin_time = time.time()

                message = client_socket.recv(self.buffer_size)  # TODO adjust buffer

                header = message[:self.header_size]
                message = message[self.header_size:]

                message_len = int(header)

                while len(message) < message_len:

                    remaining = message_len - len(message)

                    if remaining < self.buffer_size:
                        last = client_socket.recv(remaining)
                    else:
                        last = client_socket.recv(self.buffer_size)

                    message += last

                packet_size = len(message)

                message = zlib.decompress(message)

                packet = pickle.loads(message)

                delay_ms = (time.time() - _begin_time) * 1000

                output_queue.put((packet, delay_ms, packet_size))

            # except ConnectionResetError:
            #     print("Server closed connection")
            #     is_running.clear()
            except pickle.UnpicklingError as e:
                print(e)
                print("Malformed data received")
            except Exception as e:
                print(type(e))
                print(e)
                is_running.clear()

        client_socket.close()

    def close(self):
        """Shuts down the server and clears all used resources"""
        self._worker_running.clear()
        self._worker.join()

    def __enter__(self):
        """This allows the ThreadedBroadcastClient to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the ThreadedBroadcastClient to be (optionally) used in python 'with' statements"""
        self.close()

    def recv_object(self):
        """
        Pops the oldest packet received from the Queue.

        Returns:
            object: The first packet in Queue
        """

        if self._worker_running.is_set():
            return self._packets_queue.get()[0]
        else:
            return None

    def recv_object_with_net_stats(self):
        """
        Pops the oldest packet received from the Queue, along with its network stats.

        Returns:
            object: The first packet in Queue
            int: Delay (in milliseconds)
            int: Original packet size (in bytes)
        """

        if self._worker_running.is_set():
            return self._packets_queue.get()  # packet, delay_ms, packet_size
        else:
            return None

    def clear_queue(self):
        """
        TODO Experimental
        Remove all packets received until this moment from the Queue to fast forward until the latest packages.
        """

        try:
            while True:
                self._packets_queue.get_nowait()
        except Empty:
            pass
