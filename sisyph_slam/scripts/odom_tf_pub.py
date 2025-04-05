#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from matplotlib import pyplot as plt
from tf_utils import *



ident_q = np.array([0, 0, 0, 1])
zero_p = np.array([0, 0, 0, 0])

map_got_time = rospy.Time()

def map_waiter_cb(map_msg: OccupancyGrid):
    map_got_time = rospy.Time.now()


if __name__ == '__main__':

    node_handler = rospy.init_node("odom_tf_pub", anonymous=False)

    tf_mo_init_msg = TransformStamped()       
    tf_mo_init_msg.header.frame_id = "map" 
    tf_mo_init_msg.child_frame_id = "odom"  
    tf_mo_init_msg.transform.rotation = Quaternion(*ident_q) 

    pub_dt = 0.05 #secs
    no_map_timeout = 10 #secs
    map_waiter = rospy.Subscriber("/map", OccupancyGrid, map_waiter_cb)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    start_time = rospy.Time().now().to_sec()

    odom_pub_stopped = False

    try:
        while not rospy.core.is_shutdown():
            maps_too_old = (rospy.Time.now().to_sec()-map_got_time.to_sec())>no_map_timeout
            starting = (rospy.Time.now().to_sec()-start_time)<no_map_timeout
            odom_pub_cond = starting or maps_too_old 
            if odom_pub_cond:
                tf_mo_init_msg.header.stamp = rospy.Time().now()
                tf_broadcaster.sendTransform(tf_mo_init_msg)
                
                if odom_pub_stopped:
                    odom_pub_stopped = False
                    rospy.loginfo("Started sending map->odom frame")
            else:
                if not odom_pub_stopped:
                    odom_pub_stopped = True
                    rospy.loginfo("Map is building! Stopped sending map->odom frame")       

            rospy.sleep(pub_dt)

    except rospy.ROSInterruptException:
        pass

