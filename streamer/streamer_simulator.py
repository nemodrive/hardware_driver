# TODO:
#  - parse JSON file: 671a87f4eb424ab6.json
#  - timestamps: phone_data -> tp
#  - IMU
#    - orientation quaternion: phone_data -> attitude
#    - gyro rate: phone_data -> rotationRate (rotationRateUnbiased??)
#    - linear acceleration: phone_data -> acceleration ( + phone_data -> gravity??) (phone_data -> userAcceleration??)
#    -
#  - GPS
#    - longitute (x) : phone_data -> longitude
#    - latitude (y) : phone_data -> latitude
#    - altitude (z) : phone_data -> altitude
#    - easting : phone_data -> easting
#    - northing : phone_data -> northing
#    -
#  - SPEED
#    - mps (meters per second): speed_data -> mps
#    - timestamps: speed_data -> tp
#    - orientation: phone_data -> trueHeading??, IMU (orientation quaternion)

import json
from typing import List, Dict, Any
import math
import pynmea2

LOGFILE_PATH = "../../car_data_collection/data/sesiune-18-11-2018/671a87f4eb424ab6.json"

TIMESTAMPS = "timestamps"
IMU_ORIENTATIONS = "imu_orientation"
IMU_ROT_RATES = "imu_rotation_rate"
IMU_LIN_ACCS = "imu_accelerations"

GPS_LONG = "gps_longitude"
GPS_LAT = "gps_lat"
GPS_ALT = "gps_alt"

GPS_EASTING = "gps_easting"
GPS_NORTHING = "gps_northing"

SPEED_MPS = "speed_value"
SPEED_TS = "speed_timestamps"


def dd_to_dms(degs):
    neg = degs < 0
    degs = (-1) ** neg * degs
    degs, d_int = math.modf(degs)
    mins, m_int = math.modf(60 * degs)
    secs = 60 * mins
    return neg, d_int, m_int, secs


class SimulatedStreamer:
    def __init__(self, log_file:str=LOGFILE_PATH):
        self.log_data = self.__parse_log_file(log_file)

    def __parse_log_file(self, log_file) -> Dict[str, Any]:
        with open(log_file) as logf:
            json_data = json.load(logf)

            log_data = {
                TIMESTAMPS: json_data["phone_data"]["tp"],

                IMU_ORIENTATIONS: json_data["phone_data"]["attitude"],
                IMU_ROT_RATES: json_data["phone_data"]["rotationRate"],
                IMU_LIN_ACCS: json_data["phone_data"]["acceleration"],

                GPS_LAT: json_data["phone_data"]["latitude"],
                GPS_LONG: json_data["phone_data"]["longitude"],
                GPS_EASTING: json_data["phone_data"]["easting"],
                GPS_NORTHING: json_data["phone_data"]["northing"],

                SPEED_TS: json_data["speed_data"]["tp"],
                SPEED_MPS: json_data["speed_data"]["mps"]
            }

            return log_data

    def get_sim_stream(self, log_data:Dict[str, Any]) -> List[Dict[str, Any]]:
        stream = []

        ts_speed_idx = 0
        ts_rest_idx = 0

        while True:
            current_ts_speed = log_data[SPEED_TS][ts_speed_idx]
            current_ts_rest = log_data[TIMESTAMPS][ts_rest_idx]

            current_ts = min(current_ts_speed, current_ts_rest)
            packet = {
                "images": {},
                "sensor_data": {},
                "datetime": current_ts
            }

            """=========== set data ==========="""
            # set IMU data
            packet["sensor_data"]["imu"] = {
                "orientation_quaternion": log_data[IMU_ORIENTATIONS][ts_rest_idx],
                "gyro_rate": log_data[IMU_ROT_RATES][ts_rest_idx],
                "linear_acceleration": log_data[IMU_LIN_ACCS][ts_rest_idx],
            }

            # set GPS data
            lat_sign, lat_deg, lat_min, lat_sec = dd_to_dms(log_data[GPS_LAT][ts_rest_idx])
            long_sign, long_deg, long_min, long_sec = dd_to_dms(log_data[GPS_LAT][ts_rest_idx])

            lat_repr = "%03d%07.4f" % (lat_deg, lat_min + lat_sec / 60.0)
            long_repr = "%03d%07.4f" % (long_deg, long_min + long_sec / 60.0)

            lat_direction = "N"
            if lat_sign < 0:
                lat_direction = "S"
            long_direction = "E"
            if long_sign < 0:
                long_direction = "W"

            packet["sensor_data"]["gps"] = {
                "GGA": pynmea2.GGA('GP', 'GGA', (str(current_ts), lat_repr, lat_direction, long_repr, long_direction,
                                                 '1', '04', '2.6', "%f" % log_data[GPS_ALT][ts_rest_idx],
                                                 'M', '35.8953', 'M', '', '0000'))
            }

            # set speed data
            packet["sensor_data"]["speed"] = {
                "mps": log_data[SPEED_MPS][ts_speed_idx]
            }
            """=========== done set data ==========="""

            stream.append(packet)

            # increment simulated speed and GPS/IMU indexes, such that if speed readings are slower, the
            # same speed will be input with the next inertial package
            if ts_speed_idx == len(log_data[SPEED_TS]) - 1:
                if ts_rest_idx == len(log_data[TIMESTAMPS]) - 1:
                    # we have consumed all the data from the log, so we can return the resulting packet list
                    break
                else:
                    # keep reading inertial logs until the end
                    ts_rest_idx += 1
            else:
                if ts_rest_idx == len(log_data[TIMESTAMPS]) - 1:
                    # if the phone logs have finished but there are remaining speed readings, ignore them
                    break
                else:
                    # if both speed and inertial measurements remain
                    next_speed_ts = log_data[SPEED_TS][ts_speed_idx + 1]
                    next_rest_ts = log_data[TIMESTAMPS][ts_rest_idx + 1]

                    if next_rest_ts <= current_ts_speed:
                        ts_rest_idx += 1
                    elif next_rest_ts <= next_speed_ts:
                        ts_rest_idx += 1
                    else:
                        ts_speed_idx += 1
                        ts_rest_idx += 1

        return stream
