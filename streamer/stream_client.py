import signal
import yaml

from network.threaded_client import ThreadedBroadcastClient

client_running = True


def _handle_signal(signum, frame):
    global client_running
    print("Client now stopping...")
    client_running = False


signal.signal(signal.SIGINT, _handle_signal)


def main():

    with open("config.yaml", "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)["network_streaming"]

    with ThreadedBroadcastClient("192.168.100.128", settings["port"], settings["header_size"],
                                 settings["recv_buffer"]) as client:

        while client_running:

            recv_obj = client.recv_object()

            # TODO optionally decompress images

            print(recv_obj["datetime"])

if __name__ == '__main__':
    main()
