import rospy
from nav_msgs.msg import Odometry as message_type
#import pyrosbag as prb
import rosbag
#from gmplot import gmplot
import matplotlib.pyplot as plt
import argparse

# topic_name = 'gps/filtered'
# topic_name = 'test/odom_from_gps'
# bag_name ='/home/alex/.ros/nemo_fusion.bag'
bag_path='/home/dan/.ros/nemo_fusion.bag'
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# def listener():
#
#     rospy.init_node('listener', anonymous=True)
#
#     rospy.Subscriber(topic_name, message_type ,callback)
#
#     rospy.spin()

def plot_bag(bag_name, real_gps_topic, filtered_gps_topic):
    bag = rosbag.Bag(bag_name)
    # gmap = gmplot.GoogleMapPlotter(44.435179, 26.047837, 18)
    lat_filtered = []
    long_filtered = []
    lat_real =[]
    long_real = []
    for topic, msg, _ in bag.read_messages():
        # print(topic)
        if topic == filtered_gps_topic:
            x = msg.latitude
            y = msg.longitude
            lat_filtered.append(x)
            long_filtered.append(y)
            print("filtered", x, y)
        if topic == real_gps_topic:
            x = msg.latitude
            y = msg.longitude
            print ("real", x, y)
            # exit(0)
            lat_real.append(x)
            long_real.append(y)
    # gmap.scatter(lat_filtered, long_filtered, 'cornflowerblue', edge_width=10)
    # gmap.scatter(lat_real, long_real, 'red', edge_width=10)
    # gmap.draw('map-covariance-v3.html')
    fig = plt.figure("Ground truth and Filtered GPS comparison")
    ax1 = fig.add_subplot(111)

    ax1.scatter(lat_filtered, long_filtered, c='b', marker='o', label='Filtered')
    plt.scatter(lat_real, long_real, c='r', marker="+", label='Real')
    plt.legend(loc='upper left');
    bag.close()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-b', '--bag', dest='bag_file', type=str, default=bag_path,
                        help="ROS bag topic containing the filtered gps positions")
    parser.add_argument('-r', '--real', dest='real_gps_topic', type=str, default='/fix',
                        help="ROS bag topic containing the ground truth gps positions")
    parser.add_argument('-f', '--filtered', dest='filtered_gps_topic', type=str, default="/gps/filtered",
                        help="ROS bag topic containing the filtered gps positions")

    args = parser.parse_args()

    plot_bag(args.bag_file, args.real_gps_topic, args.filtered_gps_topic)
