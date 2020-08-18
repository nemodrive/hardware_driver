import unittest
import numpy as np

from .compressor import Compressor
from .decompressor import Decompressor


class TestCompressor(unittest.TestCase):

    def test_basic_compression(self):

        packet1 = {
            "images": {
                "center": np.ones((7, 7, 3))
            },
            "position": {
                "x": 12,
                "y": 10,
                "z": 8,
            }
        }

        packet2 = {
            "images": {
                "center": np.ones((7, 7, 3))
            },
            "position": {
                "x": 2,
                "y": 3,
                "z": 8,
            }
        }

        def _tmp_generator():
            for p in [packet1, packet2]:
                yield p

        compressed_generator = Compressor(_tmp_generator()).compressed_generator()

        packet1_comp = next(compressed_generator)
        packet2_comp = next(compressed_generator)

        self.assertTrue("images" not in packet2)
        self.assertEqual(packet2_comp["position"]["x"], 2)
        self.assertEqual(packet2_comp["position"]["y"], 3)
        self.assertTrue("z" not in packet2_comp["position"])

    def test_image_changed(self):

        packet1 = {
            "images": {
                "center": np.ones((7, 7, 3))
            },
            "position": {
                "x": 12,
                "y": 10,
                "z": 8,
            }
        }

        packet2 = {
            "images": {
                "center": np.ones((7, 7, 3))
            },
            "position": {
                "x": 2,
                "y": 3,
                "z": 8,
            }
        }

        packet2["images"]["center"][0][0][0] = 7

        def _tmp_generator():
            for p in [packet1, packet2]:
                yield p

        compressed_generator = Compressor(_tmp_generator()).compressed_generator()

        packet1_comp = next(compressed_generator)
        packet2_comp = next(compressed_generator)

        self.assertTrue("images" in packet2)
        self.assertEqual(packet2_comp["position"]["x"], 2)
        self.assertEqual(packet2_comp["position"]["y"], 3)
        self.assertTrue("z" not in packet2_comp["position"])

    def test_basic_decompression(self):

        packet1 = {
            "images": {
                "center": np.ones((7, 7, 3))
            },
            "position": {
                "x": 12,
                "y": 10,
                "z": 8,
            }
        }

        packet2 = {
            "images": {
                "center": np.ones((7, 7, 3))
            },
            "position": {
                "x": 2,
                "y": 3,
                "z": 8,
            }
        }

        # TODO packet 3

        def _tmp_generator():
            for p in [packet1, packet2]:
                yield p

        compressed_generator = Compressor(_tmp_generator()).compressed_generator()

        decompressed_generator = Decompressor(compressed_generator).uncompressed_generator()

        packet1_uncomp = next(decompressed_generator)
        packet2_uncomp = next(decompressed_generator)

        self.assertTrue("images" in packet2_uncomp)
        self.assertEqual(packet1_uncomp["position"]["x"], 12)
        self.assertEqual(packet1_uncomp["position"]["y"], 10)
        self.assertEqual(packet2_uncomp["position"]["x"], 2)
        self.assertEqual(packet2_uncomp["position"]["y"], 3)
        self.assertTrue("z" in packet2_uncomp["position"])

    # TODO more tests!!!


if __name__ == '__main__':
    unittest.main()
