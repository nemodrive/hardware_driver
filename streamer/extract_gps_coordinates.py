from recording.recorders import Player
import pynmea2
from tqdm import tqdm
import pandas as pd


if __name__ == '__main__':

    points = {
        "LAT": [],
        "LON": []
    }

    with Player("./saved_datasets/_c_test_automatica_1") as p:
        source_stream = p.stream_generator(loop=False)

        for packet in tqdm(source_stream):

            if packet["sensor_data"]["gps"] is not None:

                gps_data = packet["sensor_data"]["gps"]

                for sentence in gps_data.values():

                    assert isinstance(sentence, pynmea2.TalkerSentence)

                    if sentence.sentence_type == "GGA":

                        points["LAT"].append(sentence.latitude)
                        points["LON"].append(sentence.longitude)

    df = pd.DataFrame(points)

    df.to_csv("./saved_datasets/_c_test_automatica_1/gps_positions.csv", index=False)

    print("lat max: ", df["LAT"].max())
    print("lat min: ", df["LAT"].min())

    print("lon max: ", df["LON"].max())
    print("lon min: ", df["LON"].min())
