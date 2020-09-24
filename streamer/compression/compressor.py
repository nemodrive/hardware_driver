import numpy as np
from copy import deepcopy
from typing import Iterator


class Compressor:
    """
    Wraps a generator like the one in the Streamer class
    and removes redundant repeated data from consecutive packets,
    in preparation for saving on disk or streaming over the network.
    """

    def __init__(self, source_generator: Iterator[dict]):
        """
        Instantiates the wrapper with a target generator containing redundant data

        Args:
            source_generator (Iterator[dict]): target generator
        """

        self.source_generator = source_generator
        self.last_packet = None

    def _prune_dict(self, reference: dict, target: dict):
        """
        Removes elements in the target dict which already exist in the reference dict.
        TODO WARNING the two dicts will change after the function call

        Args:
            reference (dict): reference generator
            target (dict): target generator
        """

        to_delete = []

        for k, v in target.items():
            if isinstance(v, dict):
                self._prune_dict(reference[k], target[k])
            else:
                if isinstance(v, np.ndarray) and np.array_equal(v, reference[k]):
                    to_delete.append(k)
                    # del target[k]
                elif not isinstance(v, np.ndarray) and k in reference and v == reference[k]:
                    to_delete.append(k)
                    # del target[k]
                else:
                    reference[k] = deepcopy(v)

        for k, v in target.items():
            if isinstance(v, dict) and len(v) == 0:
                to_delete.append(k)

        for k in to_delete:
            del target[k]

    def compressed_generator(self) -> Iterator[dict]:
        """
        Generator that returns compressed data.

         Returns:
            Iterator[dict]: Compressed generator
        """

        for data_packet in self.source_generator:

            if self.last_packet is None:
                self.last_packet = deepcopy(data_packet)
                yield data_packet
            else:
                self._prune_dict(self.last_packet, data_packet)
                yield data_packet


class JITCompressor:
    """
    Wraps a generator like the one in the Streamer class
    and removes redundant repeated data from consecutive packets,
    in preparation for saving on disk or streaming over the network.
    """

    def __init__(self):
        """
        Instantiates the wrapper with a target generator containing redundant data
        """

        self.last_packet = None

    def _prune_dict(self, reference: dict, target: dict):
        """
        Removes elements in the target dict which already exist in the reference dict.
        TODO WARNING the two dicts will change after the function call

        Args:
            reference (dict): reference generator
            target (dict): target generator
        """

        to_delete = []

        for k, v in target.items():
            if isinstance(v, dict):
                self._prune_dict(reference[k], target[k])
            else:
                if isinstance(v, np.ndarray) and np.array_equal(v, reference[k]):
                    to_delete.append(k)
                    # del target[k]
                elif not isinstance(v, np.ndarray) and k in reference and v == reference[k]:
                    to_delete.append(k)
                    # del target[k]
                else:
                    reference[k] = deepcopy(v)

        for k, v in target.items():
            if isinstance(v, dict) and len(v) == 0:
                to_delete.append(k)

        for k in to_delete:
            del target[k]

    def compress_next_packet(self, source_packet) -> dict:
        """
        Returns compressed data.
         # TODO make the other extend this class

         Returns:
            dict: Compressed packet
        """

        if self.last_packet is None:
            self.last_packet = deepcopy(source_packet)
            return source_packet
        else:
            source_packet = deepcopy(source_packet)
            self._prune_dict(self.last_packet, source_packet)
            return source_packet
