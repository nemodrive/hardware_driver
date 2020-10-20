import signal  # ensure graceful exit
import yaml
import cv2
import logging

from player import DatasetPlayer
from network.threaded_server import ThreadedBroadcastServer
from recording.recorders import Player
from compression.decompressor import Decompressor


server_running = True


def _handle_signal(signum, frame):
    global server_running
    print("Server now stopping...")
    server_running = False


signal.signal(signal.SIGINT, _handle_signal)


def main():

    with open("config.yaml", "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)["network_streaming"]


    # tmp_img_paths = {
    #     "left": "C:/Users/Zen/Documents/GitHub/hardware_driver/streamer/legacy_code/camera/calibration_images/img_calib_demo_prime-20200813T075522Z-001/img_calib_demo_prime/DEV_000F315CDD58_2.jpg",
    #     "center": "C:/Users/Zen/Documents/GitHub/hardware_driver/streamer/legacy_code/camera/calibration_images/img_calib_demo_prime-20200813T075522Z-001/img_calib_demo_prime/DEV_000F315CD617_2.jpg",
    #     "right": "C:/Users/Zen/Documents/GitHub/hardware_driver/streamer/legacy_code/camera/calibration_images/img_calib_demo_prime-20200813T075522Z-001/img_calib_demo_prime/DEV_000F315CDD5B_2.jpg"
    # }

    # tmp_imgs = {}
    #
    # for pos, path in tmp_img_paths.items():
    #     tmp_imgs[pos] = cv2.imread(path)

    with ThreadedBroadcastServer(settings["host"], settings["port"], settings["header_size"]) as server:

        with Player() as p:

            uncomp_generator = Decompressor(p.stream_generator(True)).uncompressed_generator()

            for data_packet in uncomp_generator:

                if not server_running:
                    break

                for pos in data_packet["images"].keys():  # ["center", "left", "right"]:

                    img = data_packet["images"][pos]
                    # img = tmp_imgs[pos]

                    img = cv2.resize(img, (int(img.shape[1] / 2.8), int(img.shape[0] / 2.8)))

                    retval, data_packet["images"][pos] = cv2.imencode(".jpg", img)

                server.send_object(data_packet)


if __name__ == '__main__':
    main()
