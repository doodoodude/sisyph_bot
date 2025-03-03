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
from robot_multi_tf_pub import SisyphStatePublisher


if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    robot_tf_publisher = SisyphStatePublisher(node_handler, 46, 42, [0.2555, 0.078])

    try:
        rospy.spin()
        # while not rospy.core.is_shutdown():
    except rospy.ROSInterruptException:
        pass

