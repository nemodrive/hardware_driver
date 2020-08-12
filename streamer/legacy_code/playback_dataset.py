import yaml
import cv2
import json
import numpy as np
import zlib
import pickle
from datetime import datetime

with open("dataset_shake.json", "r") as f:
    dataset = json.load(f)

for frame in dataset:

    for pos in frame["images"].keys():
        frame["images"][pos] = pickle.loads(zlib.decompress(bytes.fromhex(frame["images"][pos])))

    frame["datetime"] = datetime.fromtimestamp(frame["datetime"])

    print(frame["sensor_data"]["gps"])

    img = frame["images"]["center"]

    img = cv2.resize(img, (int(img.shape[1] / 2), int(img.shape[0] / 2)))

    cv2.imshow("center", img)
    cv2.waitKey(0)
