#!/usr/bin/env python3

import numpy as np
import rospy
from time import sleep
from sensor_msgs.msg import LaserScan
import tf2_ros



class ScanTFSync:

    def __init__(self, nh):
        self.nh = nh
        
        self.lidar_scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=50)
        self.lidar_correct_pub = rospy.Publisher("/corrected_scan", LaserScan, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, 100)

        self.lidar_msg = LaserScan()



    def scan_callback(self, lidar_msg: LaserScan):


        self.lidar_msg = lidar_msg
        self.lidar_msg.time_increment = 0.0
        self.lidar_msg.header.stamp = rospy.Time.now()

        try:
            tf_ = self.tf_buffer.lookup_transform("robot","odom", rospy.Time(0), rospy.Duration(0.2))
            if abs(tf_.header.stamp.to_sec()-self.lidar_msg.header.stamp.to_sec())>0.2: return
            self.lidar_msg.header.stamp = tf_.header.stamp
            self.lidar_correct_pub.publish(self.lidar_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass



if __name__ == '__main__':
    node_handler = rospy.init_node("uss_to_scan", anonymous=False)
    reconstructor = ScanTFSync(node_handler)
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
