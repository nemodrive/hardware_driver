from typing import Iterator
from copy import deepcopy


class Decompressor:
    """
    Wraps generators of the Compressor class.
    Adds back the redundant data that was removed during the compression process.
    """

    def __init__(self, source_generator: Iterator[dict]):
        """
        Instantiates the wrapper with a target generator containing compressed data

        Args:
            source_generator (Iterator[dict]): target generator
        """

        self.source_generator = source_generator
        self.last_packet = None

    def _grow_dict(self, reference: dict, target: dict):
        """
        Updates the target dict with data from the reference dict,
         if a new version of it is not already in target.

        Args:
            reference (dict): reference generator
            target (dict): target generator
        """

        for k, v in reference.items():

            if k not in target:
                target[k] = deepcopy(v)
            elif isinstance(v, dict):
                self._grow_dict(reference[k], target[k])
            else:
                reference[k] = deepcopy(target[k])

        for k, v in target.items():
            if k not in reference:
                reference[k] = deepcopy(v)

    def uncompressed_generator(self) -> Iterator[dict]:
        """
        Generator that returns the decompressed data.

        Returns:
            Iterator[dict]: Decompressed generator
        """

        for data_packet in self.source_generator:

            if self.last_packet is None:
                self.last_packet = deepcopy(data_packet)
                yield data_packet
            else:
                self._grow_dict(self.last_packet, data_packet)
                yield data_packet


class JITDecompressor:
    """
    Wraps generators of the Compressor class.
    Adds back the redundant data that was removed during the compression process.
    """

    def __init__(self):
        """
        Instantiates the wrapper
        """

        self.last_packet = None

    def _grow_dict(self, reference: dict, target: dict):
        """
        Updates the target dict with data from the reference dict,
         if a new version of it is not already in target.

        Args:
            reference (dict): reference generator
            target (dict): target generator
        """

        for k, v in reference.items():

            if k not in target:
                target[k] = deepcopy(v)
            elif isinstance(v, dict):
                self._grow_dict(reference[k], target[k])
            else:
                reference[k] = deepcopy(target[k])

        for k, v in target.items():
            if k not in reference:
                reference[k] = deepcopy(v)

    def decompress_next_packet(self, source_packet) -> dict:
        """
        Generator that returns the decompressed data.

        Returns:
            dict: Decompressed packet
        """

        if self.last_packet is None:
            self.last_packet = deepcopy(source_packet)
            return source_packet
        else:
            self._grow_dict(self.last_packet, source_packet)
            return source_packet
