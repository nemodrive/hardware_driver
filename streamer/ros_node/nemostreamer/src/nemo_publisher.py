#!/usr/bin/env python
import socket
import pickle
import yaml
import rospy
import os
import time

from std_msgs.msg import String, Header
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from nmea_msgs.msg import Sentence

host = '192.168.100.128'  # '192.168.183.130'
port = 6366
HEADER_SIZE = 10
BUFFER_SIZE = 16


def publisher():

    script_path = os.path.dirname(os.path.realpath(__file__))  # can be started from any directory using rosrun

    with open(os.path.join(script_path, "config.yaml"), "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)

    string_publisher = rospy.Publisher('nemo_streamer', String, queue_size=settings["queue_size"])
    imu_publisher = rospy.Publisher('nemo_imu', Imu, queue_size=settings["queue_size"])
    nmea_publisher = rospy.Publisher('nmea_sentence', Sentence, queue_size=settings["queue_size"]) # TODO make sure nobody is publishing to this same topic

    client_socket = socket.socket()

    try:
        client_socket.connect((host, port))
    except socket.error as e:
        rospy.logerr(e)

    while not rospy.is_shutdown():

        try:
            _begin_time = time.time()

            message = client_socket.recv(BUFFER_SIZE)  # TODO adjust buffer

            header = message[:HEADER_SIZE]
            message = message[HEADER_SIZE:]

            # print("new msg len:", header)
            message_len = int(header)

            while len(message) < message_len:

                remaining = message_len - len(message)

                if remaining < BUFFER_SIZE:
                    last = client_socket.recv(remaining)
                else:
                    last = client_socket.recv(BUFFER_SIZE)

                message += last

            packet = pickle.loads(message)  # TODO request that server skips sending images

            # packet was received, preparing to send multiple ROS messages derived from it

            # TEST
            # ros_msg = String()
            # ros_msg.data = str(packet)
            # string_publisher.publish(ros_msg)
            # rospy.loginfo(str(packet))

            nemo_header = Header()  # will be used for all messages which require headers
            nemo_header.stamp = rospy.Time.now()  # TODO get from packet timestamp

            # send IMU message http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

            imu_packet = packet["sensor_data"]["imu"]

            imu_msg = Imu(
                header=nemo_header,
                orientation=Quaternion(
                    x=imu_packet["orientation_quaternion"]["x"],
                    y=imu_packet["orientation_quaternion"]["y"],
                    z=imu_packet["orientation_quaternion"]["z"],
                    w=imu_packet["orientation_quaternion"]["w"]),
                orientation_covariance=settings["imu"]["orientation_covariance"],
                angular_velocity=Vector3(
                    x=imu_packet["gyro_rate"]["x"],
                    y=imu_packet["gyro_rate"]["y"],
                    z=imu_packet["gyro_rate"]["z"]),
                angular_velocity_covariance=settings["imu"]["angular_velocity_covariance"],
                linear_acceleration=Vector3(
                    x=(imu_packet["linear_acceleration"]["x"] / 9.80665),  # g's to m/s^2
                    y=(imu_packet["linear_acceleration"]["y"] / 9.80665),  # g's to m/s^2
                    z=(imu_packet["linear_acceleration"]["z"]) / 9.80665),  # g's to m/s^2
                linear_acceleration_covariance=settings["imu"]["linear_acceleration_covariance"],
            )

            imu_publisher.publish(imu_msg)
            rospy.loginfo(str(imu_msg))

            # send GPS data to nmea_topic_driver of nmea_navsat_driver package

            for msg_type, msg_str in packet["sensor_data"]["gps"].items():

                # TODO caching might flood the ROS nmea_driver package, inspect this

                nmea_msg = Sentence(
                    header=nemo_header, # TODO The header.stamp should correspond to the time that the message was read from the device for accurate time_reference output
                    sentence=msg_str,
                )

                nmea_publisher.publish(nmea_msg)
                # rospy.loginfo(str(nmea_msg))

            _end_time = time.time()

            rospy.loginfo("FPS: " + str(1/(_end_time - _begin_time)))

        except Exception as e:
            rospy.loginfo(type(e))
            rospy.loginfo(e)
            break


if __name__ == "__main__":
    rospy.init_node("nemo_publisher")
    publisher()
