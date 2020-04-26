#!/usr/bin/env python
import socket
import pickle
import rospy

from std_msgs.msg import String

host = '192.168.100.128'  #'192.168.183.130'
port = 6366
HEADER_SIZE = 10
BUFFER_SIZE = 16


def publisher():

    pub = rospy.Publisher('nemo_streamer', String, queue_size=10)

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

            ros_msg = String()
            ros_msg.data = str(packet)

            pub.publish(ros_msg)

            rospy.loginfo(str(packet))

        except Exception as e:
            rospy.loginfo(type(e))
            rospy.loginfo(e)
            break


if __name__ == "__main__":
    rospy.init_node("nemo_publisher")
    publisher()
