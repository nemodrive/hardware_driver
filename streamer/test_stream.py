import cv2

from combined import SimpleStreamer
import logging

logging.basicConfig(level=logging.DEBUG)

grabber = SimpleStreamer()

for packet in grabber.stream_generator():

    for pos, image in packet["images"].items():
        cv2.imshow(pos, image)
