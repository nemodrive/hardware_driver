import json
import zlib
import pickle
from datetime import datetime
import time
import itertools
from copy import deepcopy


class DatasetPlayer:

    def __init__(self, dataset_path):

        with open(dataset_path, "r") as f:
            self.dataset = json.load(f)

    def stream_generator(self, loop=False):

        for frame in self.dataset:

            for pos in frame["images"].keys():
                frame["images"][pos] = pickle.loads(zlib.decompress(bytes.fromhex(frame["images"][pos])))

            frame["datetime"] = datetime.fromtimestamp(frame["datetime"])

            yield deepcopy(frame)  # TODO everywhere else we have generators too

        if loop:
            for frame in itertools.cycle(self.dataset):
                time.sleep(0.001)
                yield deepcopy(frame)


if __name__ == '__main__':

    ds = DatasetPlayer("./recordings/dataset_shake.json")

    last_time = time.time()

    for frame in ds.stream_generator(loop=True):
        print("delay", time.time() - last_time)
        print(frame["datetime"])
        last_time = time.time()
