#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis
# from sklearn.preprocessing import normalize

rospy.init_node("set_global_frame")

tf_broadcaster = tf2_ros.TransformBroadcaster()

def fiducial_transforms_cb(fid_msg: FiducialTransformArray): #массив преобразования
    # print(f"Got msg: {fid_msg.header.frame_id}")
    tf_msg = TransformStamped() #создание сообщения для преобразования координат
    tf_msg.header.frame_id = "usb_cam" #глобальный frame world
    tf_msg.header = fid_msg.header #копирование заголовка сообщения
    
    for seq, fid_tf in enumerate(fid_msg.transforms):  #перебор всех обнаруженных маркеров 

        tf_msg.child_frame_id = "map" #наследованный frame
        
        tf_msg.transform.rotation = fid_tf.transform.rotation
        # tf_msg.transform.rotation.w = -tf_msg.transform.rotation.w

        tf_msg.transform.translation.x = fid_tf.transform.translation.x
        tf_msg.transform.translation.y = fid_tf.transform.translation.y
        tf_msg.transform.translation.z = fid_tf.transform.translation.z

        tf_broadcaster.sendTransform(tf_msg)
        

fid_listener = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_transforms_cb)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass