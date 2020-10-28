#!/usr/bin/env python
import socket
import pickle
import yaml
import rospy
import os
import time
import zlib

from std_msgs.msg import String, Header
from geometry_msgs.msg import Quaternion, Vector3, Twist, Pose, Point, PoseWithCovariance, TwistWithCovariance
from sensor_msgs.msg import Imu
from nmea_msgs.msg import Sentence
from nav_msgs.msg import Odometry
import logging

from threaded_client import ThreadedBroadcastClient

host = '0.0.0.0' # '192.168.100.128'  # '192.168.183.130'
port = 6366
HEADER_SIZE = 10
HEADER_SIZE = 10
BUFFER_SIZE = 104857600


def publisher():
    logging.info("Launching NEMO Publisher")
    rospy.loginfo("Launching NEMO Publisher")

    script_path = os.path.dirname(os.path.realpath(__file__))  # can be started from any directory using rosrun

    with open(os.path.join(script_path, "config.yaml"), "r") as f:
        settings = yaml.load(f, Loader=yaml.SafeLoader)

    string_publisher = rospy.Publisher('nemo_streamer', String, queue_size=settings["queue_size"])
    imu_publisher = rospy.Publisher('nemo_imu', Imu, queue_size=settings["queue_size"])
    nmea_publisher = rospy.Publisher('nemo_nmea_sentence', Sentence, queue_size=settings["queue_size"])
    speed_publisher = rospy.Publisher('nemo_odom', Odometry, queue_size=settings["queue_size"])


    with ThreadedBroadcastClient(host, port, HEADER_SIZE, BUFFER_SIZE) as client:

        while not rospy.is_shutdown():

            #try:
            _begin_time = time.time()

            packet = client.recv_object()

            rospy.loginfo("Got packet!")
            
            # TODO request that server skips sending images

            # packet was received, preparing to send multiple ROS messages derived from it

            # nemo_header = Header()  # will be used for all messages which require headers
            # nemo_header.stamp = rospy.Time.now()  # TODO get from packet timestamp
            # nemo_header.stamp = rospy.Time.from_sec(packet["datetime"])

            # send IMU message http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
            imu_packet = packet["sensor_data"]["imu"]

            if imu_packet:
                rospy.logdebug(imu_packet)

                imu_header = Header(frame_id="imu", stamp=rospy.Time.from_sec(time.mktime(packet["datetime"].timetuple())))
                imu_msg = Imu(
                    # header=nemo_header,
                    header=imu_header,
                    orientation=Quaternion(
                        x=imu_packet["orientation_quaternion"]["x"],
                        y=imu_packet["orientation_quaternion"]["z"],
                        z=imu_packet["orientation_quaternion"]["y"],
                        w=imu_packet["orientation_quaternion"]["w"]),
                    orientation_covariance=settings["imu"]["orientation_covariance"],
                    angular_velocity=Vector3(
                        x=imu_packet["gyro_rate"]["x"],
                        y=imu_packet["gyro_rate"]["z"],
                        z=imu_packet["gyro_rate"]["y"]),
                    angular_velocity_covariance=settings["imu"]["angular_velocity_covariance"],
                    linear_acceleration=Vector3(
                        x=(imu_packet["linear_acceleration"]["x"] / 9.80665),  # g's to m/s^2
                        y=(imu_packet["linear_acceleration"]["z"] / 9.80665),  # g's to m/s^2
                        z=(imu_packet["linear_acceleration"]["y"]) / 9.80665),  # g's to m/s^2
                    linear_acceleration_covariance=settings["imu"]["linear_acceleration_covariance"],
                )

                imu_publisher.publish(imu_msg)

            # send GPS data to nmea_topic_driver of nmea_navsat_driver package
            if "gps" in packet["sensor_data"] and packet["sensor_data"]["gps"]:
                for msg_type, msg_data in packet["sensor_data"]["gps"].items():

                    # TODO caching might flood the ROS nmea_driver package, inspect this
                    # nmea_msg = Sentence(
                    #     header=nemo_header, # TODO The header.stamp should correspond to the time that the message was read from the device for accurate time_reference output
                    #     sentence=msg_str,
                    # )

                    # NOTE: it may be necessary to convert other message types to GGA type
                    # (as this is required by navsat_transform)
                    if msg_type == "GGA":
                        #rospy.loginfo(msg_data["msg"])
                        gps_header = Header(frame_id="gps", stamp=rospy.Time.from_sec(time.mktime(packet["datetime"].timetuple())))
                        nmea_msg = Sentence(header=gps_header, sentence=str(msg_data))
                        nmea_publisher.publish(nmea_msg)

            # send speed data to nemo_odometry topic

            if "canbus" in packet["sensor_data"] and packet["sensor_data"]["canbus"]:
                
                if "speed" in packet["sensor_data"]["canbus"] and packet["sensor_data"]["canbus"]["speed"] :

                    speed_data = packet["sensor_data"]["canbus"]["speed"]
                    mps = speed_data["value"]

                    odom_header = Header(frame_id="odom", stamp=rospy.Time.from_sec(time.mktime(packet["datetime"].timetuple())))
                    odom_msg = Odometry(header=odom_header)
                    odom_msg.child_frame_id = "base_link"

                    # pose will not be used, so we are setting it to constant zero
                    odom_msg.pose = PoseWithCovariance(pose=Pose(position=Point(0.0, 0.0, 0.0),
                                                                 orientation=Quaternion(0, 0, 0, 0)))

                    # for the twist we are currently specifying the speed only on the X axis
                    # TODO: the speed value needs to be broken down based on the IMU or GPS values (or difference of values)
                    odom_msg.twist = TwistWithCovariance(twist=Twist(linear=Vector3(mps, 0, 0),
                                                                     angular=Vector3(0, 0, 0)),
                                                         covariance=[0.5, 0.,0., 0.,0.,0., # lin x
                                                                     0.,0.5, 0., 0.,0.,0., # lin y
                                                                     0.,0.,0.5, 0., 0.,0., # lin z
                                                                     0.,0.,0., 0.5, 0.,0., # ang x
                                                                     0.,0.,0., 0.,0.5, 0., # ang y
                                                                     0.,0.,0., 0.,0.,0.5]) # ang z
                    speed_publisher.publish(odom_msg)

            _end_time = time.time()

            rospy.loginfo("FPS: " + str(1/(_end_time - _begin_time)))

            #except Exception as e:
            #    rospy.loginfo(type(e))
            #    rospy.loginfo(e)
            #    break

        rospy.loginfo("NEMO Publisher node has stopped ...")


if __name__ == "__main__":
    rospy.init_node("nemo_publisher", disable_signals=True)
    publisher()
