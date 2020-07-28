import cv2
from streamer import SharedMemStreamer
import logging

logging.basicConfig(level=logging.DEBUG)

if __name__ == '__main__':

    stream = SharedMemStreamer()

    for packet in stream.stream_generator():

        print(packet)

        for pos, image in packet["images"].items():
            cv2.imshow(pos, image)
            cv2.waitKey(1)
