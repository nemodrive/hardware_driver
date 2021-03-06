from typing import Iterator, Optional, Tuple, List
from copy import deepcopy
from multiprocessing import Process, Manager, Queue, Event, Pipe, connection
import os

import numpy as np
import pickle
import yaml
import cv2  # TODO test dual backend using scikit video?

from player import DatasetPlayer
from compression.compressor import Compressor, JITCompressor
from compression.decompressor import Decompressor, JITDecompressor
from streamer import SharedMemStreamer

import faster_fifo


class VideoWriteBuffer:
    """Wrapper for the chosen video encoder/writer backend, which also keeps track of the written frame indices."""

    def __init__(self, path: str, resolution: Tuple[int, int]):
        """
        Instantiates the buffer with the parameters of the video that will be recorded.
        # TODO add possibility to record using raw video codecs?

        Args:
            path (str): Directory in which the video will be saved
            resolution (Tuple[int, int]): Resolution of the frames in the video
        """

        self.path = path
        self.resolution = resolution

        self._fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._video_writer = cv2.VideoWriter(self.path, self._fourcc, 100.0, self.resolution)

        # self._fourcc = cv2.VideoWriter_fourcc(*'avc1')
        # self._video_writer = cv2.VideoWriter(self.path, self._fourcc, 100.0, self.resolution)


        self._crt_frame = 0

    def write_frame(self, frame: np.ndarray):
        """
        Append a new frame to the video on disk.

        Args:
            frame (numpy.ndarray): Frame as OpenCV format image
        """

        self._video_writer.write(frame)
        self._crt_frame += 1

        return self._crt_frame - 1

    def close(self):
        """Closes the video file and cleans all used resources."""

        self._video_writer.release()


class VideoReadBuffer:
    """Wrapper for the chosen video player backend, which also keeps track of the read frame indices."""

    def __init__(self, path: str):
        """
        Instantiates the buffer with the parameters of the video that will be played.

        Args:
            path (str): Path to the video file
        """

        self.path = path

        self._video_capture = cv2.VideoCapture(self.path)

        self.resolution = (
            self._video_capture.get(cv2.CAP_PROP_FRAME_WIDTH),
            self._video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )

        self._crt_frame = 0

    def set_frame(self, frame_number: int):
        """
        Go to a specific frame in the video.

        Args:
            frame_number (int): Zero indexed frame number
        """

        self._video_capture.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
        self._crt_frame = frame_number

    def read_frame(self) -> np.ndarray:
        """
        Get the next frame in the video.
        The frame will be returned and the buffer will advance by one frame.

        Returns:
            np.ndarray: Frame as OpenCV format image
        """

        res, frame = self._video_capture.read()
        self._crt_frame += 1
        return frame

    def get_crt_frame_number(self) -> int:
        """
        Get index of the current frame.
        This is the index of the frame that will be returned when read_frame() is called

        Returns:
            int: Zero indexed frame number
        """
        return self._crt_frame

    def close(self):
        """Closes the video file and cleans all used resources."""
        self._video_capture.release()


class Recorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Images will be saved in a separate video file for each camera POV.
    The rest of the data will be appended to a binary file using pickle.
    The first object in the file is a dict which specifies the names of the video files for each camera.
    Subsequent calls to pickle.load() will return the data packets.
    If packets contain images they will specify for each camera an index of a frame,
     which has to be extracted from the appropriate video file.
    """

    def __init__(self, out_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Recorder with the details of the dataset that will be recorded.
        To be ready for recording start() needs to be called.
        This is done automatically if the Recorder is called within a Python "with" statement.

        Args:
            out_path (Optional[str]): Directory where the dataset will be saved on disk
        """

        self.out_path = out_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = self.settings["camera_ids"].keys()
        self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        self.open_videos = {}
        self.metadata_file = None

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """

        if not os.path.exists(self.out_path):
            os.makedirs(self.out_path)

        for pos in self.enabled_positions:
            path = os.path.join(self.out_path, f"{pos}.mp4")
            self.open_videos[pos] = VideoWriteBuffer(path, self.resolution)

        self.metadata_file = open(os.path.join(self.out_path, "metadata.pkl"), "wb")

        video_paths = {}

        for pos, video_writer in self.open_videos.items():
            video_paths[pos] = os.path.basename(video_writer.path)

        pickle.dump(video_paths, self.metadata_file)

    def close(self):
        """Closes video and metadata files and cleans all used resources."""

        self.metadata_file.close()

        for video_writer in self.open_videos.values():
            video_writer.close()

    def __enter__(self):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.close()

    def record_packet(self, packet: dict):
        """
        Appends a new packet to the dataset on disk.

        Args:
            packet (dict): Data packet as provided by Streamer type objects.
        """

        packet = deepcopy(packet)  # making sure no one edits it later

        if "images" in packet.keys():
            images = packet["images"]
            saved_images = {}

            for pos, img in images.items():
                frame_index = self.open_videos[pos].write_frame(img)
                saved_images[pos] = frame_index

            new_packet = deepcopy(packet)
            new_packet["images"] = saved_images
        else:
            new_packet = deepcopy(packet)

        pickle.dump(new_packet, self.metadata_file)


class ThreadedRecorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Images will be saved in a separate video file for each camera POV.
    The rest of the data will be appended to a binary file using pickle.
    The first object in the file is a dict which specifies the names of the video files for each camera.
    Subsequent calls to pickle.load() will return the data packets.
    If packets contain images they will specify for each camera an index of a frame,
     which has to be extracted from the appropriate video file.
    """

    def __init__(self, out_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Recorder with the details of the dataset that will be recorded.
        To be ready for recording start() needs to be called.
        This is done automatically if the Recorder is called within a Python "with" statement.

        Args:
            out_path (Optional[str]): Directory where the dataset will be saved on disk
        """

        self.out_path = out_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = list(self.settings["camera_ids"].keys())
        self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        manager = Manager()

        self.packets_queue = manager.Queue()
        self._worker_running = manager.Event()

        self.worker = Process(target=self._recorder_thread,
                                args=(self.out_path, self.packets_queue, self.enabled_positions, self.resolution, self._worker_running))

    def _recorder_thread(self, out_path: str, packets_queue: Queue, enabled_positions: List[int], resolution: Tuple[int], running: Event):

        if not os.path.exists(out_path):
            os.makedirs(out_path)

        open_videos = {}

        for pos in enabled_positions:
            path = os.path.join(out_path, f"{pos}.mp4")
            open_videos[pos] = VideoWriteBuffer(path, resolution)

        metadata_file = open(os.path.join(out_path, "metadata.pkl"), "wb")

        video_paths = {}

        for pos, video_writer in open_videos.items():
            video_paths[pos] = os.path.basename(video_writer.path)

        pickle.dump(video_paths, metadata_file)

        while running.is_set():
            # TODO

            packet = packets_queue.get()

            packet = deepcopy(packet)  # making sure no one edits it later

            if "images" in packet.keys():
                images = packet["images"]
                saved_images = {}

                for pos, img in images.items():
                    frame_index = open_videos[pos].write_frame(img)
                    saved_images[pos] = frame_index

                new_packet = deepcopy(packet)
                new_packet["images"] = saved_images
            else:
                new_packet = deepcopy(packet)

            pickle.dump(new_packet, metadata_file)

        metadata_file.close()

        for video_writer in open_videos.values():
            video_writer.close()

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """

        self._worker_running.set()
        self.worker.start()

    def close(self):
        """Closes video and metadata files and cleans all used resources."""

        self._worker_running.clear()
        self.worker.join()

    def __enter__(self):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.close()

    def record_packet(self, packet: dict):
        """
        Appends a new packet to the dataset on disk.

        Args:
            packet (dict): Data packet as provided by Streamer type objects.
        """

        # TODO multiple queues? im1, im2, im3 and data

        self.packets_queue.put(packet)


class PipedRecorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Images will be saved in a separate video file for each camera POV.
    The rest of the data will be appended to a binary file using pickle.
    The first object in the file is a dict which specifies the names of the video files for each camera.
    Subsequent calls to pickle.load() will return the data packets.
    If packets contain images they will specify for each camera an index of a frame,
     which has to be extracted from the appropriate video file.
    """

    def __init__(self, out_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Recorder with the details of the dataset that will be recorded.
        To be ready for recording start() needs to be called.
        This is done automatically if the Recorder is called within a Python "with" statement.

        Args:
            out_path (Optional[str]): Directory where the dataset will be saved on disk
        """

        self.out_path = out_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = list(self.settings["camera_ids"].keys())
        self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        manager = Manager()

        self.packets_pipe_send, packets_pipe_recv = Pipe() #manager.Pipe()
        self._worker_running = manager.Event()

        self.worker = Process(target=self._recorder_thread,
                                args=(self.out_path, packets_pipe_recv, self.enabled_positions, self.resolution, self._worker_running))

    def _recorder_thread(self, out_path: str, packets_pipe_recv: connection.Connection, enabled_positions: List[int], resolution: Tuple[int], running: Event):

        if not os.path.exists(out_path):
            os.makedirs(out_path)

        open_videos = {}

        for pos in enabled_positions:
            path = os.path.join(out_path, f"{pos}.mp4")
            open_videos[pos] = VideoWriteBuffer(path, resolution)

        metadata_file = open(os.path.join(out_path, "metadata.pkl"), "wb")

        video_paths = {}

        for pos, video_writer in open_videos.items():
            video_paths[pos] = os.path.basename(video_writer.path)

        pickle.dump(video_paths, metadata_file)

        while running.is_set():
            # TODO

            packet_orig = packets_pipe_recv.recv()

            packet = deepcopy(packet_orig)  # making sure no one edits it later

            if "images" in packet.keys():
                images = packet["images"]
                saved_images = {}

                for pos, img in images.items():
                    frame_index = open_videos[pos].write_frame(img)
                    saved_images[pos] = frame_index

                new_packet = deepcopy(packet)
                new_packet["images"] = saved_images
            else:
                new_packet = deepcopy(packet)

            pickle.dump(new_packet, metadata_file)

        metadata_file.close()

        for video_writer in open_videos.values():
            video_writer.close()

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """

        self._worker_running.set()
        self.worker.start()

    def close(self):
        """Closes video and metadata files and cleans all used resources."""

        self._worker_running.clear()
        self.worker.join()

    def __enter__(self):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.close()

    def record_packet(self, packet: dict):
        """
        Appends a new packet to the dataset on disk.

        Args:
            packet (dict): Data packet as provided by Streamer type objects.
        """

        # TODO multiple queues? im1, im2, im3 and data

        self.packets_pipe_send.send(packet)


class Player:
    """Plays back a dataset recorded using a Recorder object"""

    def __init__(self, in_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Player with the details of the dataset that will be played back.
        To be ready for playback start() needs to be called.
        This is done automatically if the Player is called within a Python "with" statement.

        Args:
            in_path (Optional[str]): Directory where the dataset is found on disk
        """

        self.in_path = in_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = self.settings["camera_ids"].keys()
        # self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        self.open_videos = {}
        self.metadata_file = None

    def start(self):
        """
        Opens the video files and makes sure the Player is ready to stream data.
        Is called automatically by __enter__() if the Player is called within a Python "with" statement.
        """

        self.metadata_file = open(os.path.join(self.in_path, "metadata.pkl"), "rb")

        video_paths = pickle.load(self.metadata_file)

        for pos in self.enabled_positions:
            self.open_videos[pos] = VideoReadBuffer(os.path.join(self.in_path, video_paths[pos]))

    def close(self):
        """Closes video and metadata files and cleans all used resources."""
        self.metadata_file.close()

        for video_reader in self.open_videos.values():
            video_reader.close()

    def __enter__(self):
        """This allows the Player to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Player to be (optionally) used in Python 'with' statements"""
        self.close()

    def get_next_packet(self) -> dict:
        """
        Return the next packet in the recording.
        Recording will advance to the next packet after get_next_packet() is called.

        Returns:
            dict: Read data packet.
        """

        try:
            packet_small = pickle.load(self.metadata_file)
        except (EOFError, pickle.UnpicklingError):
            return None

        if "images" in packet_small:
            # a packet with images, get them from the videos

            packet_big = deepcopy(packet_small)

            for pos, img_num in packet_small["images"].items():

                if not img_num == self.open_videos[pos].get_crt_frame_number():
                    print("ERROR Frame index differs from video index!!!!")

                img = self.open_videos[pos].read_frame()
                packet_big["images"][pos] = img

            return packet_big
        else:
            return packet_small

    def rewind(self):
        """Rewinds the dataset, the next packet returned by the stream will be the first packet in the dataset"""

        self.metadata_file.seek(0, 0)

        video_paths = pickle.load(self.metadata_file)

        for pos in self.enabled_positions:
            self.open_videos[pos].set_frame(0)

    def stream_generator(self, loop: Optional[bool] = False) -> Iterator[dict]:
        """
        Wraps get_next_packet() in the form of a generator for convenience.

        Args:
            loop (Optional[bool]): If true will perform rewind() when the dataset reaches its end.

        Returns:
            Iterator[dict]: Generator providing played back packets

        """

        while True:
            while packet := self.get_next_packet():
                yield packet

            if loop:
                self.rewind()
                continue
            else:
                break


class FastRecorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Images will be saved in a separate video file for each camera POV.
    The rest of the data will be appended to a binary file using pickle.
    The first object in the file is a dict which specifies the names of the video files for each camera.
    Subsequent calls to pickle.load() will return the data packets.
    If packets contain images they will specify for each camera an index of a frame,
     which has to be extracted from the appropriate video file.
    """

    def __init__(self, out_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Recorder with the details of the dataset that will be recorded.
        To be ready for recording start() needs to be called.
        This is done automatically if the Recorder is called within a Python "with" statement.

        Args:
            out_path (Optional[str]): Directory where the dataset will be saved on disk
        """

        self.out_path = out_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = list(self.settings["camera_ids"].keys())
        self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        manager = Manager()

        self.packets_queue = faster_fifo.Queue(100000 * 1000)
        self._worker_running = manager.Event()

        self.worker = Process(target=self._recorder_thread,
                                args=(self.out_path, self.packets_queue, self.enabled_positions, self.resolution, self._worker_running))

    def _recorder_thread(self, out_path: str, packets_queue: faster_fifo.Queue, enabled_positions: List[int], resolution: Tuple[int], running: Event):

        if not os.path.exists(out_path):
            os.makedirs(out_path)

        open_videos = {}

        for pos in enabled_positions:
            path = os.path.join(out_path, f"{pos}.mp4")
            open_videos[pos] = VideoWriteBuffer(path, resolution)

        metadata_file = open(os.path.join(out_path, "metadata.pkl"), "wb")

        video_paths = {}

        for pos, video_writer in open_videos.items():
            video_paths[pos] = os.path.basename(video_writer.path)

        pickle.dump(video_paths, metadata_file)

        while running.is_set():
            # TODO

            packet = packets_queue.get()

            packet = deepcopy(packet)  # making sure no one edits it later

            if "images" in packet.keys():
                images = packet["images"]
                saved_images = {}

                for pos, img in images.items():
                    frame_index = open_videos[pos].write_frame(img)
                    saved_images[pos] = frame_index

                new_packet = deepcopy(packet)
                new_packet["images"] = saved_images
            else:
                new_packet = deepcopy(packet)

            pickle.dump(new_packet, metadata_file)

        metadata_file.close()

        for video_writer in open_videos.values():
            video_writer.close()

        print("Recorder thread has closed all open files!")

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """

        self._worker_running.set()
        self.worker.start()

    def close(self):
        """Closes video and metadata files and cleans all used resources."""

        self._worker_running.clear()
        self.worker.join()

    def __enter__(self):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.close()

    def record_packet(self, packet: dict):
        """
        Appends a new packet to the dataset on disk.

        Args:
            packet (dict): Data packet as provided by Streamer type objects.
        """

        # TODO multiple queues? im1, im2, im3 and data

        self.packets_queue.put(packet)


class FastSeparateRecorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Images will be saved in a separate video file for each camera POV.
    The rest of the data will be appended to a binary file using pickle.
    The first object in the file is a dict which specifies the names of the video files for each camera.
    Subsequent calls to pickle.load() will return the data packets.
    If packets contain images they will specify for each camera an index of a frame,
     which has to be extracted from the appropriate video file.
    """

    def __init__(self, out_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Recorder with the details of the dataset that will be recorded.
        To be ready for recording start() needs to be called.
        This is done automatically if the Recorder is called within a Python "with" statement.

        Args:
            out_path (Optional[str]): Directory where the dataset will be saved on disk
        """

        self.out_path = out_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = list(self.settings["camera_ids"].keys())
        self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        if not os.path.exists(out_path):
            os.makedirs(out_path)  # before anything else, just to make sure

        video_paths = {}

        for pos in self.enabled_positions:
            video_paths[pos] = f"{pos}.mp4"

        manager = Manager()

        self.packets_queue = faster_fifo.Queue(100000 * 1000)
        self._worker_running = manager.Event()

        self.worker = Process(target=self._recorder_thread,
                                args=(self.out_path, self.packets_queue, video_paths, self._worker_running))

        self.open_videos = {}

    def _recorder_thread(self, out_path: str, packets_queue: faster_fifo.Queue, video_paths: dict, running: Event):

        metadata_file = open(os.path.join(out_path, "metadata.pkl"), "wb")

        pickle.dump(video_paths, metadata_file)

        while running.is_set():

            packet = packets_queue.get()

            if packet is None:
                break  # send None to stop proc

            pickle.dump(packet, metadata_file)

        metadata_file.close()

        print("Recorder thread has closed all open files!")

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """

        self._worker_running.set()
        self.worker.start()

        for pos in self.enabled_positions:
            path = os.path.join(self.out_path, f"{pos}.mp4")
            self.open_videos[pos] = VideoWriteBuffer(path, self.resolution)  # excluding them from child process

    def close(self):
        """Closes video and metadata files and cleans all used resources."""

        for video_writer in self.open_videos.values():
            video_writer.close()

        print("closed video writers")

        self._worker_running.clear()
        self.packets_queue.put(None)
        self.worker.join()

    def __enter__(self):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.close()

    def record_packet(self, packet: dict):
        """
        Appends a new packet to the dataset on disk.

        Args:
            packet (dict): Data packet as provided by Streamer type objects.
        """

        # TODO multiple queues? im1, im2, im3 and data

        packet = deepcopy(packet)  # making sure no one edits it later

        if "images" in packet.keys():
            images = packet["images"]
            saved_images = {}

            for pos, img in images.items():
                frame_index = self.open_videos[pos].write_frame(img)
                saved_images[pos] = frame_index

            packet["images"] = saved_images

        # print(packet["images"])

        self.packets_queue.put(packet)


class FastCompressedRecorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Images will be saved in a separate video file for each camera POV.
    The rest of the data will be appended to a binary file using pickle.
    The first object in the file is a dict which specifies the names of the video files for each camera.
    Subsequent calls to pickle.load() will return the data packets.
    If packets contain images they will specify for each camera an index of a frame,
     which has to be extracted from the appropriate video file.
    """

    def __init__(self, out_path: Optional[str] = "./test_recording/"):
        """
        Instantiates the Recorder with the details of the dataset that will be recorded.
        To be ready for recording start() needs to be called.
        This is done automatically if the Recorder is called within a Python "with" statement.

        Args:
            out_path (Optional[str]): Directory where the dataset will be saved on disk
        """

        self.out_path = out_path

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.enabled_positions = list(self.settings["camera_ids"].keys())
        self.resolution = (self.settings["video_resolution"]["width"], self.settings["video_resolution"]["height"])

        manager = Manager()

        self.packets_queue = faster_fifo.Queue(1024 ** 3)  # 1 GB buffer
        self._worker_running = manager.Event()

        self.worker = Process(target=self._recorder_thread,
                                args=(self.out_path, self.packets_queue, self.enabled_positions, self.resolution, self._worker_running))

    def _recorder_thread(self, out_path: str, packets_queue: faster_fifo.Queue, enabled_positions: List[int], resolution: Tuple[int], running: Event):

        if not os.path.exists(out_path):
            os.makedirs(out_path)

        open_videos = {}

        for pos in enabled_positions:
            path = os.path.join(out_path, f"{pos}.mp4")
            open_videos[pos] = VideoWriteBuffer(path, resolution)

        metadata_file = open(os.path.join(out_path, "metadata.pkl"), "wb")

        video_paths = {}

        for pos, video_writer in open_videos.items():
            video_paths[pos] = os.path.basename(video_writer.path)

        pickle.dump(video_paths, metadata_file)

        jit_compressor = JITCompressor()

        while running.is_set():
            # TODO

            packet = packets_queue.get()

            packet = deepcopy(packet)  # making sure no one edits it later

            packet = jit_compressor.compress_next_packet(packet)

            if "images" in packet.keys():
                images = packet["images"]
                saved_images = {}

                for pos, img in images.items():
                    frame_index = open_videos[pos].write_frame(img)
                    saved_images[pos] = frame_index

                new_packet = deepcopy(packet)
                new_packet["images"] = saved_images
            else:
                new_packet = deepcopy(packet)

            pickle.dump(new_packet, metadata_file)

        metadata_file.close()

        for video_writer in open_videos.values():
            video_writer.close()

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """

        self._worker_running.set()
        self.worker.start()

    def close(self):
        """Closes video and metadata files and cleans all used resources."""

        self._worker_running.clear()
        self.worker.join()

    def __enter__(self):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """This allows the Recorder to be (optionally) used in Python 'with' statements"""
        self.close()

    def record_packet(self, packet: dict):
        """
        Appends a new packet to the dataset on disk.

        Args:
            packet (dict): Data packet as provided by Streamer type objects.
        """

        # TODO multiple queues? im1, im2, im3 and data

        self.packets_queue.put(packet)


if __name__ == '__main__':

    streamer = SharedMemStreamer()
    # TODO give it a warmup period?
    source_stream = streamer.stream_generator()

    # with Player("./saved_datasets/recording_test") as p:
    #
    #     source_stream = Decompressor(p.stream_generator(loop=True)).uncompressed_generator()

    with FastSeparateRecorder(out_path="./saved_datasets/asjhdajhsdjhasd") as r:

        jit_compressor = JITCompressor()

        for i in range(100):

            recv_obj = next(source_stream)

            compressed = jit_compressor.compress_next_packet(
                recv_obj)  # TODO compress inside the recorder, decompress inside the player, with threads

            r.record_packet(compressed)
