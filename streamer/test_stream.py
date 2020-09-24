import cv2
from streamer import SharedMemStreamer
import logging
import time

logging.basicConfig(level=logging.DEBUG)

if __name__ == '__main__':

    stream = SharedMemStreamer()

    last_time = time.time()

    # TODO beware of https://github.com/skvark/opencv-python/issues/362

    # cv2.namedWindow('center', cv2.WINDOW_AUTOSIZE)
    # cv2.namedWindow('left', cv2.WINDOW_AUTOSIZE)
    # cv2.namedWindow('right', cv2.WINDOW_AUTOSIZE)

    # cv2.startWindowThread()

    for packet in stream.stream_generator():

        # print(packet)

        fps = 1 / (time.time() - last_time)
        last_time = time.time()

        print(fps)
        print(packet["speed"])

        # print(packet["images"].keys())

        # cv2.imshow("center", packet["images"]["center"])
        # cv2.imshow("left", packet["images"]["left"])
        # cv2.imshow("right", packet["images"]["right"])
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # for pos, image in packet["images"].items():

            # print(image.shape)

            # frame_small = cv2.resize(image, (int(image.shape[1] / 2), int(image.shape[0] / 2)))
            #
            # cv2.imshow(pos, frame_small)
            # cv2.waitKey(1)
            # cv2.destroyAllWindows()

        time.sleep(0.2)
