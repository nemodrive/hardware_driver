import socket
import pickle
import logging
import signal
import time

client_running = True


def _handle_signal(signum, frame):
    global client_running
    # print("Client now stopping...")
    client_running = False


signal.signal(signal.SIGINT, _handle_signal)


host = '127.0.0.1'  #'192.168.183.130'
port = 6366
HEADER_SIZE = 10
BUFFER_SIZE = 16

client_socket = socket.socket()

try:
    client_socket.connect((host, port))
except socket.error as e:
    logging.error(e)

while client_running:

    try:

        message = client_socket.recv(BUFFER_SIZE)  # TODO adjust buffer

        header = message[:HEADER_SIZE]
        message = message[HEADER_SIZE:]

        # print("new msg len:", header)
        message_len = int(header)

        while len(message) < message_len:

            remaining = message_len - len(message)

            if remaining < BUFFER_SIZE:
                last = client_socket.recv(remaining)
            else:
                last = client_socket.recv(BUFFER_SIZE)

            message += last

        packet = pickle.loads(message)
        print(packet)

    except ConnectionResetError:
        print("Server closed connection")
        break
    except pickle.UnpicklingError as e:
        print(e)
        print("Malformed data received")
        # break
    except Exception as e:
        print(type(e))
        print(e)

client_socket.close()
