import socket
import yaml
import logging
import time
import pickle
from multiprocessing import Process, Manager, Value, Event
from typing import List, Dict
import random
from datetime import datetime
import signal  # ensure graceful exit

from combined import SimpleStreamer

HEADER_SIZE = 10

server_running = True


def _handle_signal(signum, frame):
    global server_running
    print("Server now stopping...")
    server_running = False


signal.signal(signal.SIGINT, _handle_signal)


def accept_clients_thread(clients: List, running: Event()):

    with open("config.yaml", "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)

    main_socket = socket.socket()

    try:
        main_socket.bind((settings["car_data_server"]["host"], settings["car_data_server"]["port"]))
    except socket.error as e:
        logging.error(e)

    print('Server started, waiting for clients...')
    main_socket.listen(5)

    while running.is_set():
        client_socket, client_address = main_socket.accept()
        print('Connection request from: %s:%d' % (client_address[0], client_address[1]))

        clients.append((client_socket, client_address))

    print("worker now stopping")

    main_socket.close()


# def generate_mock_data_packet():
#     yield {
#
#         "images": {
#             "left": "A" * 1024,
#             "center": "A" * 1024,
#             "right": "A" * 1024,
#         },
#         "sensor_data": {
#             "gps":
#                 {
#                     "GGA": "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
#                     "GSA": "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39",
#                 },
#             "imu": {
#
#             }
#         },
#         "datetime": datetime.now(),
#     }


def main():

    manager = Manager()

    clients = manager.list()
    worker_running = manager.Event()

    worker_running.set()
    accept_clients_worker = Process(target=accept_clients_thread, args=(clients, worker_running))
    accept_clients_worker.start()

    grabber = SimpleStreamer()

    while server_running:

        data_packet = next(grabber.stream_generator())

        # TODO don't send images if client does not request them

        message = pickle.dumps(data_packet, protocol=2)  # TODO ROS only supports python2

        header = bytes(("%-" + str(HEADER_SIZE) + "d") % len(message), 'utf-8')

        # print(header)

        message = header + message

        disconnected_clients = []

        for client in clients:
            # send it to all clients
            # TODO nonblocking send via twisted?
            # TODO timeit!

            client_socket, client_address = client

            try:
                client_socket.sendall(message)
            except (ConnectionAbortedError, ConnectionResetError):
                print("Client at %s:%d has aborted connection, removing from queue" % client_address)
                disconnected_clients.append(client_address)
            except Exception as e:
                print(type(e))
                print(e)

        if len(disconnected_clients) > 0:
            for i, c in enumerate(clients):
                if c[1] in disconnected_clients:
                    del clients[i]  # remove does not work on proxies only del does

    worker_running.clear()
    print("waiting for worker to join")
    accept_clients_worker.terminate()  # TODO join not working, investigate


if __name__ == '__main__':
    main()
