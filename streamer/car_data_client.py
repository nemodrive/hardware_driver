import socket
import pickle
import logging
import yaml
import signal

client_running = True


def _handle_signal(signum, frame):
    global client_running
    # print("Client now stopping...")
    client_running = False


signal.signal(signal.SIGINT, _handle_signal)


host = '127.0.0.1'
port = 6366

client_socket = socket.socket()

try:
    client_socket.connect((host, port))
except socket.error as e:
    logging.error(e)

while client_running:

    try:
        response = client_socket.recv(1024)  # TODO adjust buffer
        packet = pickle.loads(response)
        print(packet)
    except ConnectionResetError:
        print("Server closed connection")
        break
    except pickle.UnpicklingError:
        print("Malformed data received, shutting down!")
        break
    except Exception as e:
        print(type(e))
        print(e)

client_socket.close()
