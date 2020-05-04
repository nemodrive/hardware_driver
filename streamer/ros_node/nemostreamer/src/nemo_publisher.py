#!/usr/bin/env python
import socket
import pickle
import rospy

from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

host = '192.168.100.128'  # '192.168.183.130'
port = 6366
HEADER_SIZE = 10
BUFFER_SIZE = 16


def publisher():
    string_publisher = rospy.Publisher('nemo_streamer', String, queue_size=10)
    imu_publisher = rospy.Publisher('nemo_streamer', Imu, queue_size=10)

    client_socket = socket.socket()

    try:
        client_socket.connect((host, port))
    except socket.error as e:
        rospy.logerr(e)

    while not rospy.is_shutdown():

        try:
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

            packet = pickle.loads(message)

            # packet was received, preparing to send multiple ROS messages derived from it

            # TEST
            # ros_msg = String()
            # ros_msg.data = str(packet)
            # string_publisher.publish(ros_msg)
            # rospy.loginfo(str(packet))

            nemo_header = Header()  # will be used for all messages which require headers
            nemo_header.stamp = rospy.Time.now()

            # send IMU message http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

            imu_msg = Imu(
                header=nemo_header,
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                orientation_covariance=[0.99] * 9,
                angular_velocity=Vector3(x=0.1, y=0.2, z=0.0),
                angular_velocity_covariance=[0.99] * 9,
                linear_acceleration=Vector3(x=0.0, y=0.1, z=0.0),
                linear_acceleration_covariance=[0.99] * 9,
            )

            imu_publisher.publish(imu_msg)
            rospy.loginfo(str(imu_msg))

        except Exception as e:
            rospy.loginfo(type(e))
            rospy.loginfo(e)
            break


if __name__ == "__main__":
    rospy.init_node("nemo_publisher")
    publisher()