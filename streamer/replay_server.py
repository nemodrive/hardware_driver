import signal  # ensure graceful exit
import yaml
import cv2
import logging

from player import DatasetPlayer
from network.threaded_server import ThreadedBroadcastServer


server_running = True


def _handle_signal(signum, frame):
    global server_running
    print("Server now stopping...")
    server_running = False


signal.signal(signal.SIGINT, _handle_signal)


def main():

    streamer = DatasetPlayer("recordings/dataset_shake.json")

    with open("config.yaml", "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)["network_streaming"]

    with ThreadedBroadcastServer(settings["host"], settings["port"], settings["header_size"]) as server:

        for data_packet in streamer.stream_generator(loop=True):

            if not server_running:
                break

            for pos in data_packet["images"].keys():

                img = data_packet["images"][pos]

                img = cv2.resize(img, (int(img.shape[1] / 2.8), int(img.shape[0] / 2.8)))

                retval, data_packet["images"][pos] = cv2.imencode(".jpg", img)

            server.send_object(data_packet)


if __name__ == '__main__':
    main()
