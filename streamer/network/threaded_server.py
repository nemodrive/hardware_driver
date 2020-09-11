import logging
import zlib
import socket, select
import multiprocessing
import queue
from multiprocessing import Process, Manager, Event, Queue
from multiprocessing.managers import SyncManager
from typing import List, Tuple
from math import log10

import pickle

logger = multiprocessing.get_logger()
logger.setLevel(logging.INFO)
logger.propagate = False

NEMO_STREAM_MANAGER_KEY = "nemo".encode()


class NemoStreamManager(SyncManager):
    """
    Implements the manager process that synchronizes the data collection process with the
    processes serving individual clients (e.g. GUI, recorder, ROS Publisher. It maintains:
      - the list of client workers (i.e. subscribed processes that deliver messages from the shared memory streamer
      - the flag that specifies that the shared memory collector process is still working
    """
    def __init__(self, address=('0.0.0.0', 8986), authkey=NEMO_STREAM_MANAGER_KEY):
        super(NemoStreamManager, self).__init__(address=address, authkey=authkey)


class ThreadedBroadcastServer:
    """
    Receives requests to broadcast a Python object to its connected clients.
    Each client is allocated its own worker process and message Queue for fast operation.
    """

    # def __init__(self, sync_manager_address, host: str, port: int, header_size: int):
    def __init__(self, collector_running: multiprocessing.Event, stream_workers,
                 sync_manager_host, sync_manager_port,
                 host: str, port: int, header_size: int):
        """
        Instantiates the server and starts the worker process which accepts connections on the specified port.

        Args:
            sync_manager: The multiprocessing.Manager object used to sync between the sensor data collector process
            and the distributor processes that connect to it via sockets
            host (str): Server host address
            port (int): Server port where new connection requests will be sent
            header_size (int): Number of bytes used to represent object size
        """

        self.host = host
        self.port = port
        self.header_size = header_size

        self._client_acceptor_running = collector_running
        self._stream_workers = stream_workers

        # create and start the client listener process
        # self._accept_clients_worker = Process(target=self._accept_clients_thread, args=(self.sync_manager.address,))
        self._accept_clients_worker = Process(target=self._accept_clients_thread,
                                              args=(collector_running, stream_workers))
        self._accept_clients_worker.start()

    def __create_manager(self):
        return Manager()

    def _accept_clients_thread(self, collector_running, stream_workers):
    # def _accept_clients_thread(self, sync_manager_address):
        """
        Blueprint method for the thread which listens on the main port and opens a connection for each new client.

        Args:
            # client_workers (List): A list shared by the multiprocessing Manager where the new clients will be added
            # running (Event): The process will shut down when this event is cleared
            sync_manager_address: address for the sync_manager Manager from which the run flag and the shared list
            of clients is obtained
        """
        # create listening socket
        main_socket = socket.socket()
        main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        main_socket.settimeout(0.2)

        try:
            main_socket.bind((self.host, self.port))
        except socket.error as e:
            logger.error(e)

        logger.info('Server started, waiting for clients...')
        main_socket.listen(5)
        main_socket.setblocking(False)

        child_workers: List[Tuple[Process, Queue, Event]] = []

        local_worker_manager = self.__create_manager()

        while collector_running.is_set():
            readable, writable, err = select.select([main_socket.fileno()], [], [], 1)
            if readable:
                client_socket, client_address = main_socket.accept()  # this is why it will not join

                logger.info('Connection request from: %s:%d' % (client_address[0], client_address[1]))

                child_worker_running = local_worker_manager.Event()
                child_worker_running.set()
                sender_queue = local_worker_manager.Queue()

                sender_worker = Process(target=self._send_to_client_thread,
                                        args=[sender_queue, client_socket, client_address, child_worker_running])
                sender_worker.start()

                stream_workers.append((sender_queue, child_worker_running))
                child_workers.append((sender_worker, sender_queue, child_worker_running))

        logger.info("Stopping client threads...")

        for _, _, child_worker_running in child_workers:
            child_worker_running.clear()

        # for _, sender_queue, child_worker_running in child_workers:
        #     sender_queue.join_thread()

        for w, _, child_worker_running in child_workers:
            w.join()

        # local_worker_manager.shutdown()

        # when the collector_running flag is false, it means no data can be forwarded any more
        # so we can
        main_socket.close()

        logger.info("Client acceptor shutting down!")

    def _send_to_client_thread(self, packet_queue: Queue, destination_socket: socket.socket,
                               client_address: Tuple[str, int], is_running: Event):
        """
        Blueprint method for a thread that is dedicated to one of the clients.

        Args:
            packet_queue (Queue): Packets which need to be sent to the client will be added here
            destination_socket (socket.socket): Client socket
            client_address Tuple[str, int]: Client host and port
            is_running: The process will shut down when this event is cleared
        """
        while is_running.is_set():
            try:
                crt_packet = packet_queue.get(timeout=1)  # TODO analyze queue sizes during operation

                destination_socket.sendall(crt_packet)
                packet_queue.task_done()
            except queue.Empty as e:
                logger.debug("No data within one second; Retrying.")
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                logger.info("Client at %s:%d has aborted connection, removing from queue" % client_address)
                is_running.clear()
            except Exception as e:
                logger.error(type(e))
                logger.error(e)
                is_running.clear()

        # packet_queue.close()
        logger.info("Stream worker for client %s has terminated" % str(destination_socket.getsockname()))

    def object_to_bytes(self, obj):
        """
        Converts a Python object to a Bytes object that can be sent over the network.

        Args:
            obj (object): The object that will be sent
        """

        message = pickle.dumps(obj, protocol=2)  # TODO ROS only supports python2

        message = zlib.compress(message)  # not much compression, just to make sure repeated data won't cause lag

        if int(log10(len(message))) + 1 > self.header_size:
            logging.error("FATAL ERROR! HEADER_SIZE OVERFLOW!")
            raise Exception("FATAL ERROR! HEADER_SIZE OVERFLOW!")
        else:
            header = bytes(("%-" + str(self.header_size) + "d") % len(message), 'utf-8')

        message = header + message

        return message

    def send_object(self, obj):
        """
        Broadcasts a Python object to all connected clients.

        Args:
            obj (object): The object that will be sent
        """

        message = self.object_to_bytes(obj)

        disconnected_clients = []

        stream_workers = self._stream_workers
        print(stream_workers)

        for i, worker_info in enumerate(stream_workers):
            # send it to all clients

            sender_queue, is_running = worker_info

            if is_running.is_set():
                # TODO if queue is very large only send when it clears up
                sender_queue.put(message)
            else:
                disconnected_clients.append(i)

        if len(disconnected_clients) > 0:
            for client_index in sorted(disconnected_clients, reverse=True):
                del stream_workers[client_index]
            disconnected_clients.clear()

    def close(self):
        """Shuts down the server and clears all used resources"""
        self._client_acceptor_running.clear()
        logger.info("###################### waiting for workers to join")
        self._accept_clients_worker.join()

    def __enter__(self):
        """This allows the ThreadedBroadcastServer to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the ThreadedBroadcastServer to be (optionally) used in python 'with' statements"""
        self.close()
