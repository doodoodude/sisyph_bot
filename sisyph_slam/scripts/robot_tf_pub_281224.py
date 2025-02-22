#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_conjugate as quat_conj
from tf.transformations import quaternion_from_euler as euler_to_quat
from tf.transformations import euler_from_quaternion as quat_to_euler
from tf.transformations import quaternion_about_axis as quat_axis
from tf.transformations import quaternion_inverse as quat_inv
from tf.transformations import quaternion_multiply as quat_mult
# from sklearn.preprocessing import normalize


def rot_by_quat(_vect, _quat):
    return quat_mult(quat_mult(_quat, _vect), quat_conj(_quat))

def get_quat_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.rotation.x, _msg.transform.rotation.y, _msg.transform.rotation.z, _msg.transform.rotation.w])
def get_trans_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.translation.x, _msg.transform.translation.y, _msg.transform.translation.z, 0.0])


correct_fid_quat = euler_to_quat(-3.1414,-3.1414,0)
ident_quat = np.array([0, 0, 0, 1])
flipz_quat = quat_axis(3.1414, (0,0,1))



class SisyphStatePublisher:

    def __init__(self, nh):

        self.nh = nh

        self.fid_robot = 46
        self.fid_world = 42

        self.no_map_timeout = 10
        self.no_map = False

        fid_listener0 = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fid_tf0_cb, queue_size=1)
        map_waiter = rospy.Subscriber("/map", OccupancyGrid, self.map_waiter_cb)

        self.tf_broadcaster0 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster1 = tf2_ros.TransformBroadcaster()
        # static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.tf_world_usbcam_msg = TransformStamped()
        self.tf_odom_robot_msg = TransformStamped()
        self.tf_world_map_init_msg = TransformStamped()

        self.tf_world_usbcam_msg.header.frame_id = "world" 
        self.tf_world_usbcam_msg.child_frame_id = "usb_cam"
        self.tf_world_usbcam_msg.transform.rotation = Quaternion(*ident_quat)

        self.tf_odom_robot_msg.header.frame_id = "odom" 
        self.tf_odom_robot_msg.child_frame_id = "robot" 
        self.tf_odom_robot_msg.transform.rotation = Quaternion(*ident_quat)

        self.tf_world_map_init_msg.header.frame_id = "world" 
        self.tf_world_map_init_msg.child_frame_id = "map"
        self.tf_world_map_init_msg.transform.rotation = Quaternion(*ident_quat)


        self.tf_robot_laser_msg = TransformStamped() # STATIC
        self.tf_robot_laser_msg.header.frame_id = "robot" 
        self.tf_robot_laser_msg.child_frame_id = "laser"  
        self.tf_robot_laser_msg.transform.rotation = Quaternion(*ident_quat)
        self.tf_robot_laser_msg.transform.translation.x = 0.2555
        self.tf_robot_laser_msg.transform.translation.y = 0.078
        
        self.tf_map_odom_init_msg = TransformStamped() # STATIC
        self.tf_map_odom_init_msg.header.frame_id = "map" 
        self.tf_map_odom_init_msg.child_frame_id = "odom"  
        self.tf_map_odom_init_msg.transform.rotation = Quaternion(*ident_quat)

        self.init_published = False
        self.world_found = False

        self.quat_world_map_init = np.array([0,0,0,1])
        self.trans_world_map_init = np.array([0,0,0,0])
        self.quat_world_map_init_inv = np.array([0,0,0,1])
        self.trans_world_map_init_inv = np.array([0,0,0,0])

        self.quat_usbcam_world_inv = np.array([0,0,0,1])
        self.trans_usbcam_world_inv = np.array([0,0,0,0])

        self.start_time = rospy.Time.now().to_sec()
        self.map_got_time = rospy.Time.now().to_sec()-self.no_map_timeout-1

        self.odom_pub_stopped = True





    def process_tf_msg(self, fid_msg: FiducialTransformArray):

        self.tf_map_odom_init_msg.header.stamp = fid_msg.header.stamp
        self.tf_robot_laser_msg.header.stamp = fid_msg.header.stamp

        self.tf_world_map_init_msg.header.stamp = fid_msg.header.stamp
        self.tf_odom_robot_msg.header.stamp = fid_msg.header.stamp
        self.tf_world_usbcam_msg.header.stamp = fid_msg.header.stamp

        for seq, fid_tf in enumerate(fid_msg.transforms):
            tf_quat = get_quat_arr_from_tf_msg(fid_tf)
            tf_trans = get_trans_arr_from_tf_msg(fid_tf)

            if fid_tf.fiducial_id == self.fid_world: # world -> usb_cam
                if not self.world_found:
                    self.world_found = True

                    self.quat_usbcam_world_inv = quat_inv(tf_quat) 
                    self.trans_usbcam_world_inv = -rot_by_quat(tf_trans, self.quat_usbcam_world_inv) 

                    self.tf_world_usbcam_msg.transform.rotation = Quaternion(*self.quat_usbcam_world_inv)
                    self.tf_world_usbcam_msg.transform.translation.x = self.trans_usbcam_world_inv[0]
                    self.tf_world_usbcam_msg.transform.translation.y = self.trans_usbcam_world_inv[1]
                    self.tf_world_usbcam_msg.transform.translation.z = self.trans_usbcam_world_inv[2]

                    rospy.loginfo("Found usb_cam base tf")

            if fid_tf.fiducial_id == self.fid_robot and self.world_found:   # world -> obot, world->map, map->odom, 
                # quat_world_robot = quaternion_multiply(self.quat_usbcam_world_inv, tf_quat)
                quat_world_robot = quat_mult(
                                    quat_mult(self.quat_usbcam_world_inv, tf_quat),
                                    flipz_quat) # doing Z flip
                quat_world_robot = quat_axis(quat_to_euler(quat_world_robot)[2], 
                                                         [0,0,1]) # using only rotation about Z

                trans_world_robot = rot_by_quat(tf_trans, self.quat_usbcam_world_inv) + self.trans_usbcam_world_inv
                trans_world_robot[2] = 0.0 # set Z to zero
            
                if not self.init_published:
                    self.init_published = True

                    self.quat_world_map_init  = quat_world_robot.copy()
                    self.trans_world_map_init  = trans_world_robot.copy()

                    self.tf_world_map_init_msg.transform.rotation = Quaternion(*self.quat_world_map_init )
                    self.tf_world_map_init_msg.transform.translation.x = self.trans_world_map_init[0]
                    self.tf_world_map_init_msg.transform.translation.y = self.trans_world_map_init[1]
                    self.tf_world_map_init_msg.transform.translation.z = self.trans_world_map_init[2]    

                    self.quat_world_map_init_inv  = quat_inv(self.quat_world_map_init ) 
                    self.trans_world_map_init_inv  = -rot_by_quat(self.trans_world_map_init , self.quat_world_map_init_inv) 
                    quat_odom_robot = ident_quat
                    trans_odom_robot = np.zeros(4)

                    rospy.loginfo(f"Found init odom tf")
                else:

                    quat_odom_robot = quat_mult(self.quat_world_map_init_inv, quat_world_robot)
                    trans_odom_robot = rot_by_quat(trans_world_robot, self.quat_world_map_init_inv) + self.trans_world_map_init_inv 

                self.tf_odom_robot_msg.transform.rotation = Quaternion(*quat_odom_robot) 
                self.tf_odom_robot_msg.transform.translation.x = trans_odom_robot[0]
                self.tf_odom_robot_msg.transform.translation.y = trans_odom_robot[1]
                self.tf_odom_robot_msg.transform.translation.z = trans_odom_robot[2]


            





    def fid_tf0_cb(self, fid_msg: FiducialTransformArray):

        # rospy.loginfo(fid_msg.header.stamp.to_sec() - rospy.Time.now().to_sec())
        # fid_msg.header.stamp = rospy.Time.now()

        if len(fid_msg.transforms)>0:

            dt_start = (rospy.Time.now().to_sec()-self.start_time)
            dt_map = (rospy.Time.now().to_sec()-self.map_got_time)

            self.process_tf_msg(fid_msg)

            if self.world_found:
                self.tf_broadcaster0.sendTransform([
                                                    self.tf_odom_robot_msg,
                                                    self.tf_world_usbcam_msg, 
                                                    self.tf_world_map_init_msg, 
                                                    self.tf_robot_laser_msg, 
                                                    ])
                
                maps_too_old = True #dt_map>self.no_map_timeout
                starting = dt_start<self.no_map_timeout
                odom_pub_cond = starting or maps_too_old or self.no_map
            
                if odom_pub_cond:
                    self.tf_broadcaster0.sendTransform(self.tf_map_odom_init_msg)
                    
                    if self.odom_pub_stopped:
                        self.odom_pub_stopped = False
                        rospy.loginfo("Started sending map->odom frame")
                else:
                    if not self.odom_pub_stopped:
                        self.odom_pub_stopped = True
                        rospy.loginfo("Map is building! Stopped sending map->odom frame")







    def map_waiter_cb(self, map_msg: OccupancyGrid):
        self.map_got_time = rospy.Time.now().to_sec()





if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    reconstructor = SisyphStatePublisher(node_handler)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
