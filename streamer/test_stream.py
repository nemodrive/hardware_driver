import cv2
from vimba import *

from combined import SimpleStreamer
import logging

logging.basicConfig(level=logging.DEBUG)

if __name__ == '__main__':

    grabber = SimpleStreamer()

    for packet in grabber.stream_generator():

        print(packet)

        for pos, image in packet["images"].items():
            cv2.imshow(pos, image)
            cv2.waitKey(1)
