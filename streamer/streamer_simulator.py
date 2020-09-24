import json
import math
import pynmea2
from datetime import datetime
import time

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
    def __init__(self, log_file=LOGFILE_PATH):
        self.log_data = self.__parse_log_file(log_file)

    def __parse_log_file(self, log_file):
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
                GPS_ALT: json_data["phone_data"]["altitude"],

                SPEED_TS: json_data["speed_data"]["tp"],
                SPEED_MPS: json_data["speed_data"]["mps"]
            }

            return log_data

    def stream_generator(self, time_delayed: bool = False, simulator_delay: float = None):
        ts_speed_idx = 0
        ts_idx = 0

        current_ts = self.log_data[TIMESTAMPS][ts_idx]
        current_ts_speed = self.log_data[SPEED_TS][ts_speed_idx]

        send_first_speed = False
        if current_ts_speed <= current_ts:
            send_first_speed = True

        stream_end = False

        while not stream_end:
            current_ts_speed = self.log_data[SPEED_TS][ts_speed_idx]
            current_ts = self.log_data[TIMESTAMPS][ts_idx]

            packet = {
                "images": {},
                "sensor_data": {},
                "datetime": current_ts
            }

            """=========== set data ==========="""
            # set IMU data
            packet["sensor_data"]["imu"] = {
                "orientation_quaternion": self.log_data[IMU_ORIENTATIONS][ts_idx],
                "gyro_rate": self.log_data[IMU_ROT_RATES][ts_idx],
                "linear_acceleration": self.log_data[IMU_LIN_ACCS][ts_idx],
            }
            #print(packet["sensor_data"]["imu"])

            # set GPS data
            gps_time = datetime.utcfromtimestamp(current_ts).strftime('%H%M%S.%f')[:-4]

            lat_sign, lat_deg, lat_min, lat_sec = dd_to_dms(self.log_data[GPS_LAT][ts_idx])
            long_sign, long_deg, long_min, long_sec = dd_to_dms(self.log_data[GPS_LONG][ts_idx])

            lat_repr = "%03d%07.4f" % (lat_deg, lat_min + lat_sec / 60.0)
            long_repr = "%03d%07.4f" % (long_deg, long_min + long_sec / 60.0)

            lat_direction = "N"
            if lat_sign < 0:
                lat_direction = "S"
            long_direction = "E"
            if long_sign < 0:
                long_direction = "W"

            packet["sensor_data"]["gps"] = {
                "GGA": {
                    "msg": pynmea2.GGA('GP', 'GGA', (gps_time, lat_repr, lat_direction, long_repr, long_direction,
                                                 '1', '04', '2.6', "%f" % self.log_data[GPS_ALT][ts_idx],
                                                 'M', '35.8953', 'M', '', '0000')),
                    "timestamp": current_ts
                }
            }

            # set speed data
            if send_first_speed:
                packet["sensor_data"]["speed"] = {
                    "mps": self.log_data[SPEED_MPS][ts_speed_idx]
                }
                send_first_speed = False
            else:
                if current_ts >= current_ts_speed and ts_speed_idx <= len(self.log_data[SPEED_TS]) - 1:
                    # if the IMU timestamps have caught up with the next speed ts, then send the speed and
                    # update the current speed ts
                    packet["sensor_data"]["speed"] = {
                        "mps": self.log_data[SPEED_MPS][ts_speed_idx]
                    }

                    ts_speed_idx += 1

            """=========== done set data ==========="""
            # check for termination
            if ts_idx == len(self.log_data[TIMESTAMPS]) - 1:
                stream_end = True
            else:
                if not time_delayed:
                    # progress timestamp index
                    ts_idx += 1

                    yield packet
                else:
                    if ts_idx < len(self.log_data[TIMESTAMPS]) - 1:
                        if simulator_delay is None:
                            next_ts = self.log_data[TIMESTAMPS][ts_idx + 1]
                            delta_ts_millis = (next_ts - current_ts)
                            time.sleep(delta_ts_millis)
                        else:
                            time.sleep(simulator_delay)

                        ts_idx += 1

                        yield packet

            # # increment simulated speed and GPS/IMU indexes, such that if speed readings are slower, the
            # # same speed will be input with the next inertial package
            # if ts_speed_idx == len(self.log_data[SPEED_TS]) - 1:
            #     if ts_idx == len(self.log_data[TIMESTAMPS]) - 1:
            #         # we have consumed all the data from the log, so we can return the resulting packet list
            #         break
            #     else:
            #         # keep reading inertial logs until the end
            #         ts_idx += 1
            # else:
            #     if ts_idx == len(self.log_data[TIMESTAMPS]) - 1:
            #         # if the phone logs have finished but there are remaining speed readings, ignore them
            #         break
            #     else:
            #         # if both speed and inertial measurements remain
            #         next_speed_ts = self.log_data[SPEED_TS][ts_speed_idx + 1]
            #         next_rest_ts = self.log_data[TIMESTAMPS][ts_idx + 1]
            #
            #         if next_rest_ts <= current_ts_speed:
            #             ts_idx += 1
            #         elif next_rest_ts <= next_speed_ts:
            #             ts_idx += 1
            #         else:
            #             ts_speed_idx += 1
            #             ts_idx += 1


if __name__ == "__main__":
    sim_streamer = SimulatedStreamer()
    log_data = sim_streamer.stream_generator()

    print(log_data[0])
