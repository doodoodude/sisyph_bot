#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis
# from sklearn.preprocessing import normalize

rospy.init_node("fid_to_tf")

tf_broadcaster = tf2_ros.TransformBroadcaster()

def fiducial_transforms_cb(fid_msg: FiducialTransformArray):
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = "usb_cam"
    tf_msg.header = fid_msg.header

    for seq, fid_tf in enumerate(fid_msg.transforms):

        tf_msg.child_frame_id = "odom" #f"marker_{fid_tf.fiducial_id}"

        tf_quat = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
        only_yaw = quaternion_from_euler(0,0,-euler_from_quaternion(tf_quat)[1],"rxyz")
        tf_msg.transform.rotation = Quaternion(*only_yaw)

        tf_msg.transform.translation.x = fid_tf.transform.translation.z
        tf_msg.transform.translation.y = fid_tf.transform.translation.y
        tf_broadcaster.sendTransform(tf_msg)

fid_listener = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_transforms_cb)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass