import signal  # ensure graceful exit
import yaml

from streamer import SharedMemStreamer
from network.threaded_server import ThreadedBroadcastServer


server_running = True


def _handle_signal(signum, frame):
    global server_running
    print("Server now stopping...")
    server_running = False


signal.signal(signal.SIGINT, _handle_signal)


def main():

    streamer = SharedMemStreamer()

    with open("config.yaml", "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)["network_streaming"]

    with ThreadedBroadcastServer(settings["host"], settings["port"], settings["header_size"]) as server:

        for data_packet in streamer.stream_generator():

            if not server_running:
                break

            # TODO optionally compress images

            server.send_object(data_packet)


if __name__ == '__main__':
    main()
