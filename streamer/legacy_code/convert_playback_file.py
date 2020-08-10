import yaml
import cv2
import json
import pickle
import zlib
from datetime import datetime


with open("demo_prime_stream_normal.yaml", "r") as f:
    dataset = yaml.load(f, Loader=yaml.UnsafeLoader)

for frame in dataset:
    for pos in frame["images"].keys():
        # retval, frame["images"][pos] = cv2.imencode(".jpg", frame["images"][pos])
        # frame["images"][pos] = frame["images"][pos].tostring()

        frame["images"][pos] = zlib.compress(pickle.dumps(frame["images"][pos])).hex()

    frame["datetime"] = datetime.timestamp(frame["datetime"])

    for key, value in frame.items():
        print(key, type(value))

with open("dataset_normal.json", "w") as f:
    json.dump(dataset, f)

for frame in dataset:
    print(frame["sensor_data"]["imu"])
