from recording.binary_recorders import BinaryRecorder
from streamer import Streamer

import numpy as np
import datetime
import time
import sys

import signal

terminate = False


def _handle_signal(signum, frame):
    global terminate
    terminate = True
    print("Shutting down please wait...")


signal.signal(signal.SIGINT, _handle_signal)


def generate_mock_packet():
    return {
        'images': {
            'center': np.random.randint(0, 128, size=(2064, 1544, 3), dtype=np.uint8),
            'left': np.random.randint(0, 128, size=(2064, 1544, 3), dtype=np.uint8),
            'right': np.random.randint(0, 128, size=(2064, 1544, 3), dtype=np.uint8),
        },
        'sensor_data': {
            'imu': {
                'orientation_quaternion': {
                    'x': -0.05830882862210274,
                    'y': 0.7980778813362122,
                    'z': -0.006055936217308044,
                    'w': 0.5996959209442139
                },
                'gyro_rate': {
                    'x': 0.005259210709482431,
                    'y': -0.0005344388191588223,
                    'z': -0.0006866774056106806
                },
                'linear_acceleration': {
                    'x': -0.00011169165372848511,
                    'y': -0.048743367195129395,
                    'z': -0.009797166101634502
                }
            },
            'gps': {
                'TXT': '$GNTXT,01,01,02,PF=3EA*4F'
            },
            'speed': {
                'mps': 0.0,
                'timestamp': 1600960021.958118
            }
        },
        'datetime': datetime.datetime(2020, 9, 24, 18, 7, 1, 961599)
    }


if __name__ == '__main__':

    streamer = Streamer()
    # TODO give it a warmup period?
    source_stream = streamer.stream_generator()

    # with Player("./saved_datasets/recording_test") as p:
    #
    #     source_stream = Decompressor(p.stream_generator(loop=True)).uncompressed_generator()

    # with BinaryRecorder(out_path="./saved_datasets/delete_me_2") as r:
    with BinaryRecorder(out_path="/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/tst", compress_gzip=False) as r:

        job_start_time = time.time()

        minimal_delay = 0.01  # 0.025?

        while not terminate:

            frame_start_time = time.time()

            recv_obj = next(source_stream) # generate_mock_packet()

            # print(f"packet size = {sys.getsizeof(recv_obj)}")

            r.record_packet(recv_obj)

            delay = time.time() - frame_start_time

            print(f"delay { delay} min delay {minimal_delay}")

            if delay < minimal_delay:
                # TODO limit FPS
                time.sleep(minimal_delay - delay)
                print(f"sleeping for: {minimal_delay - delay}")

            total_frametime_after_delay = time.time() - frame_start_time
            print(f"frametime: {total_frametime_after_delay} FPS: {1 / total_frametime_after_delay}")

    # print(f"Recording finished in {time.time() - job_start_time}")

    print("Finished!")
