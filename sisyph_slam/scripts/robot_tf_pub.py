#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_inverse, quaternion_multiply, quaternion_conjugate
# from sklearn.preprocessing import normalize

rospy.init_node("robot_tf_pub")

tf_broadcaster = tf2_ros.TransformBroadcaster()

ident_quat = np.array([0, 0, 0, 1])
flipz_quat = quaternion_about_axis(3.1414, (0,0,1))

tf_msg1 = TransformStamped()
tf_msg1.header.frame_id = "world" 
tf_msg1.child_frame_id = "usb_cam" 
tf_msg1.transform.rotation = Quaternion(*ident_quat)

tf_msg2 = TransformStamped()
tf_msg2.header.frame_id = "world" 
tf_msg2.child_frame_id = "robot" 
tf_msg2.transform.rotation = Quaternion(*ident_quat)

tf_robot_laser_msg = TransformStamped()
tf_robot_laser_msg.header.frame_id = "world" 
tf_robot_laser_msg.child_frame_id = "laser"  
tf_robot_laser_msg.transform.rotation = Quaternion(*flipz_quat)


correct_fid_quat = quaternion_from_euler(-3.1414,-3.1414,0)


def rotate_vector_by_quat(_vect, _quat):
    return quaternion_multiply(quaternion_multiply(_quat, _vect), quaternion_conjugate(_quat))

def get_quat_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.rotation.x, _msg.transform.rotation.y, _msg.transform.rotation.z, _msg.transform.rotation.w])
def get_trans_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.translation.x, _msg.transform.translation.y, _msg.transform.translation.z, 0.0])


def fiducial_transforms_cb(fid_msg: FiducialTransformArray):

    tf_robot_laser_msg.header.stamp = fid_msg.header.stamp
    tf_msg1.header.stamp = fid_msg.header.stamp
    tf_msg2.header.stamp = fid_msg.header.stamp

    # NEED TO LAUNCH rosrun tf static_transform_publisher 0 0 0 0 0 0 world map 5000

    for seq, fid_tf in enumerate(fid_msg.transforms):
        tf_quat = get_quat_arr_from_tf_msg(fid_tf)
        tf_trans = get_trans_arr_from_tf_msg(fid_tf)

        # if fid_tf.fiducial_id == 42: # world -> usb_cam
        #     tf_quat1_inv = quaternion_inverse(tf_quat) #quaternion_multiply(correct_fid_quat, tf_quat) 
        #     tf_trans1_inv = -rotate_vector_by_quat(tf_trans, tf_quat1_inv) 

        #     tf_msg1.transform.rotation = Quaternion(*tf_quat1_inv)
        #     tf_msg1.transform.translation.x = tf_trans1_inv[0]
        #     tf_msg1.transform.translation.y = tf_trans1_inv[1]
        #     tf_msg1.transform.translation.z = tf_trans1_inv[2]
            # tf_broadcaster.sendTransform(tf_msg1)

        # if fid_tf.fiducial_id == 46:   # usb_cam -> robot
        #     tf_msg2.transform = fid_tf.transform
        #     tf_broadcaster.sendTransform(tf_msg2)

        if fid_tf.fiducial_id == 46:   # world -> robot
            tf_quat2 = get_quat_arr_from_tf_msg(fid_tf)
            tf_trans2 = get_trans_arr_from_tf_msg(fid_tf)

            tf_quat1_inv = get_quat_arr_from_tf_msg(tf_msg1)
            tf_trans1_inv = get_trans_arr_from_tf_msg(tf_msg1)

            tf_quat2 = quaternion_multiply(tf_quat1_inv, tf_quat2)
            tf_trans2 = rotate_vector_by_quat(tf_trans2, tf_quat1_inv) + tf_trans1_inv

            tf_msg2.transform.rotation = Quaternion(*tf_quat2)
            tf_msg2.transform.translation.x = tf_trans2[0]
            tf_msg2.transform.translation.y = tf_trans2[1]
            tf_msg2.transform.translation.z = tf_trans2[2]            
            tf_broadcaster.sendTransform(tf_msg2)

        # if fid_tf.fiducial_id == 46: # odom -> usb_cam
        #     tf_quat1_inv = quaternion_inverse(tf_quat) #quaternion_multiply(correct_fid_quat, tf_quat) 
        #     tf_trans1_inv = -rotate_vector_by_quat(tf_trans, tf_quat1_inv) 

        #     tf_msg1.transform.rotation = Quaternion(*tf_quat1_inv)
        #     tf_msg1.transform.translation.x = tf_trans1_inv[0]
        #     tf_msg1.transform.translation.y = tf_trans1_inv[1]
        #     tf_msg1.transform.translation.z = tf_trans1_inv[2]

        # if fid_tf.fiducial_id == 42:   # odom -> robot
        #     tf_quat2 = get_quat_arr_from_tf_msg(fid_tf)
        #     tf_trans2 = get_trans_arr_from_tf_msg(fid_tf)

        #     tf_quat1_inv = get_quat_arr_from_tf_msg(tf_msg1)
        #     tf_trans1_inv = get_trans_arr_from_tf_msg(tf_msg1)

        #     tf_quat2 = quaternion_multiply(tf_quat1_inv, tf_quat2)
        #     tf_trans2 = rotate_vector_by_quat(tf_trans2, tf_quat1_inv) + tf_trans1_inv

        #     tf_msg2.transform.rotation = Quaternion(*tf_quat2)
        #     tf_msg2.transform.translation.x = tf_trans2[0]
        #     tf_msg2.transform.translation.y = tf_trans2[1]
        #     tf_msg2.transform.translation.z = tf_trans2[2]            
        #     tf_broadcaster.sendTransform(tf_msg2)

    # tf_quat2 = quaternion_multiply(tf_quat2, flipz_quat)
    tf_broadcaster.sendTransform(tf_msg2)

# Subscribe to the /fiducial_transforms topic
fid_listener = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_transforms_cb)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass
