from recording.recorder import FastSeparateRecorder, FastRecorder
from compression.compressor import Compressor, JITCompressor, ImageOnlyJITCompressor
from streamer import SharedMemStreamer

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

    streamer = SharedMemStreamer()
    # TODO give it a warmup period?
    source_stream = streamer.stream_generator()

    # with Player("./saved_datasets/recording_test") as p:
    #
    #     source_stream = Decompressor(p.stream_generator(loop=True)).uncompressed_generator()

    with FastSeparateRecorder(out_path="./saved_datasets/recordin_slight_speedup1") as r:

        jit_compressor = ImageOnlyJITCompressor() #JITCompressor()

        job_start_time = time.time()

        while not terminate:

            recv_obj = next(source_stream) #generate_mock_packet()

            # print(f"packet size = {sys.getsizeof(recv_obj)}")

            frame_start_time = time.time()

            compressed = jit_compressor.compress_next_packet(recv_obj)
            # compressed = recv_obj

            #print(f"Delay Compress {time.time() - frame_start_time}")

            rec_start_time = time.time()

            r.record_packet(compressed)

            #print(f"Delay record {time.time() - rec_start_time}")

            delay = time.time() - frame_start_time

            print(f"frametime: {delay} FPS: {1 / delay}")

    # print(f"Recording finished in {time.time() - job_start_time}")

    print("Finished!")
