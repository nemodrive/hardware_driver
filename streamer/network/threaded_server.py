import logging
import zlib
import socket
from multiprocessing import Process, Manager, Event, Queue
from typing import List, Tuple
from math import log10

import pickle


class ThreadedBroadcastServer:
    """
    Receives requests to broadcast a Python object to its connected clients.
    Each client is allocated its own worker process and message Queue for fast operation.
    """

    def __init__(self, host: str, port: int, header_size: int):
        """
        Instantiates the server and starts the worker process which accepts connections on the specified port.

        Args:
            host (str): Server host address
            port (int): Server port where new connection requests will be sent
            header_size (int): Number of bytes used to represent object size
        """

        self.host = host
        self.port = port
        self.header_size = header_size

        manager = Manager()

        self._client_workers = manager.list()
        self._worker_running = manager.Event()

        self._worker_running.set()
        self._accept_clients_worker = Process(target=self._accept_clients_thread,
                                              args=(self._client_workers, self._worker_running))
        self._accept_clients_worker.start()

    def _accept_clients_thread(self, client_workers: List, running: Event):
        """
        Blueprint method for the thread which listens on the main port and opens a connection for each new client.

        Args:
            client_workers (List): A list shared by the multiprocessing Manager where the new clients will be added
            running (Event): The process will shut down when this event is cleared
        """

        main_socket = socket.socket()
        main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        main_socket.settimeout(0.2)

        manager = Manager()

        try:
            main_socket.bind((self.host, self.port))
        except socket.error as e:
            logging.error(e)

        logging.info('Server started, waiting for clients...')
        main_socket.listen(5)

        child_workers = []

        while running.is_set():
            try:
                client_socket, client_address = main_socket.accept()  # this is why it will not join
                logging.info('Connection request from: %s:%d' % (client_address[0], client_address[1]))

                is_running = manager.Event()
                is_running.set()
                sender_queue = manager.Queue()
                sender_worker = Process(target=self._send_to_client_thread,
                                        args=(sender_queue, client_socket, client_address, is_running))
                sender_worker.start()

                client_workers.append((sender_queue, is_running))

                child_workers.append((sender_worker, is_running))
            except socket.timeout:
                continue

        logging.info("Stopping client threads...")

        for _, is_running in child_workers:
            is_running.clear()

        for w, is_running in child_workers:
            w.join()

        main_socket.close()

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

            crt_packet = packet_queue.get()  # TODO analyze queue sizes during operation

            try:
                destination_socket.sendall(crt_packet)
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                logging.info("Client at %s:%d has aborted connection, removing from queue" % client_address)
                is_running.clear()
            except Exception as e:
                logging.error(type(e))
                logging.error(e)
                is_running.clear()

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

        for i, worker_info in enumerate(self._client_workers):
            # send it to all clients

            sender_queue, is_running = worker_info

            if is_running.is_set():
                sender_queue.put(message)
            else:
                disconnected_clients.append(i)

        if len(disconnected_clients) > 0:
            for client_index in sorted(disconnected_clients, reverse=True):
                del disconnected_clients[client_index]
            disconnected_clients.clear()

    def close(self):
        """Shuts down the server and clears all used resources"""
        self._worker_running.clear()
        logging.debug("waiting for workers to join")
        self._accept_clients_worker.join()

    def __enter__(self):
        """This allows the ThreadedBroadcastServer to be (optionally) used in python 'with' statements"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the ThreadedBroadcastServer to be (optionally) used in python 'with' statements"""
        self.close()
