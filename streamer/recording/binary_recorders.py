from typing import Optional, Iterator
import os

import pickle
import yaml


class BinaryRecorder:
    """
    Takes in a car data generator such as the one in the Streamer class and records its output to disk for later use.
    Records straight to disk for fastest speed.
    No compression!!!
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

        self.file_path = os.path.join(self.out_path, "recording.pkl")

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.file_handle = None

        if not os.path.exists(out_path):
            os.makedirs(out_path)  # before anything else, just to make sure

    def start(self):
        """
        Opens the video files and makes sure the Recorder is ready to receive data packets.
        Is called automatically by __enter__() if the Recorder is called within a Python "with" statement.
        """
        self.file_handle = open(self.file_path, "wb")

    def close(self):
        """Closes video and metadata files and cleans all used resources."""
        self.file_handle.close()

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

        pickle.dump(packet, self.file_handle, protocol=5)  # only works in Python 3.8 !!!


class BinaryPlayer:
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

        self.file_path = os.path.join(self.in_path, "recording.pkl")

        # load settings from configuration file
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.yaml")), "r") as f:
            self.settings = yaml.load(f, Loader=yaml.SafeLoader)

        self.file_handle = None

    def start(self):
        """
        Opens the video files and makes sure the Player is ready to stream data.
        Is called automatically by __enter__() if the Player is called within a Python "with" statement.
        """
        self.file_handle = open(self.file_path, "rb")

    def close(self):
        """Closes video and metadata files and cleans all used resources."""
        self.file_handle.close()

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
            packet = pickle.load(self.file_handle)
        except (EOFError, pickle.UnpicklingError):
            packet = None

        return packet

    def rewind(self):
        """Rewinds the dataset, the next packet returned by the stream will be the first packet in the dataset"""

        self.file_handle.seek(0, 0)

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
