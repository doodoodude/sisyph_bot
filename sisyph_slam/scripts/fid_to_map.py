#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_inverse, quaternion_multiply, quaternion_conjugate
# from sklearn.preprocessing import normalize

rospy.init_node("tasty_test")

tf_broadcaster = tf2_ros.TransformBroadcaster()

ident_quat = [0, 0, 0, 1]
correct_fid_quat = quaternion_from_euler(-3.1414,-3.1414,0)

def fiducial_transforms_cb(fid_msg: FiducialTransformArray):

    tf_msg1 = TransformStamped()
    tf_msg2 = TransformStamped()

    tf_msg1.header.frame_id = "usb_cam" 
    tf_msg1.child_frame_id = "map" # from usb_cam to map  

    tf_msg2.header.frame_id = "usb_cam" 
    tf_msg2.child_frame_id = "robot" # from usb_cam to robot  

    tf_msg1.header.stamp = fid_msg.header.stamp
    tf_msg2.header.stamp = fid_msg.header.stamp

    # NEED TO LAUNCH rosrun tf static_transform_publisher 0 0 0 0 0 0 world map 5000

    for seq, fid_tf in enumerate(fid_msg.transforms):
        tf_quat = [fid_tf.transform.rotation.x, fid_tf.transform.rotation.y, fid_tf.transform.rotation.z, fid_tf.transform.rotation.w]
        tf_trans = [fid_tf.transform.translation.x, fid_tf.transform.translation.y, fid_tf.transform.translation.z, 0.0]

        if fid_tf.fiducial_id == 42:
            # tf_quat1 = tf_quat #quaternion_multiply(correct_fid_quat, tf_quat) 
            # tf_msg1.transform.rotation = Quaternion(*tf_quat1)
            # tf_msg1.transform.translation = fid_tf.transform.translation
            tf_msg1.transform = fid_tf.transform
            tf_broadcaster.sendTransform(tf_msg1)

        if fid_tf.fiducial_id == 46:
            tf_msg2.transform = fid_tf.transform
            tf_broadcaster.sendTransform(tf_msg2)


# Subscribe to the /fiducial_transforms topic
fid_listener = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_transforms_cb)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass
