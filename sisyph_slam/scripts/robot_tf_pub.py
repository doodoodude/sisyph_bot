#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_inverse, quaternion_multiply, quaternion_conjugate
# from sklearn.preprocessing import normalize


def rotate_vector_by_quat(_vect, _quat):
    return quaternion_multiply(quaternion_multiply(_quat, _vect), quaternion_conjugate(_quat))

def get_quat_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.rotation.x, _msg.transform.rotation.y, _msg.transform.rotation.z, _msg.transform.rotation.w])
def get_trans_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.translation.x, _msg.transform.translation.y, _msg.transform.translation.z, 0.0])


correct_fid_quat = quaternion_from_euler(-3.1414,-3.1414,0)
ident_quat = np.array([0, 0, 0, 1])
flipz_quat = quaternion_about_axis(3.1414, (0,0,1))



class SisyphStatePublisher:

    def __init__(self, nh):

        self.nh = nh

        fid_listener = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_transforms_cb)
        map_waiter = rospy.Subscriber("/map", OccupancyGrid, self.map_waiter_cb)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.cams = ["cam0", "cam1"]

        self.tf_world_usbcam_msg = [TransformStamped(), TransformStamped()]
        self.tf_odom_robot_msg = [TransformStamped(), TransformStamped()]
        for i_cam_, cam_ in enumerate(self.cams):
            self.tf_world_usbcam_msg[i_cam_].header.frame_id = "world" 
            self.tf_world_usbcam_msg[i_cam_].child_frame_id = cam_
            self.tf_world_usbcam_msg[i_cam_].transform.rotation = Quaternion(*ident_quat)
 
            self.tf_odom_robot_msg[i_cam_].header.frame_id = "odom" 
            self.tf_odom_robot_msg[i_cam_].child_frame_id = "robot" 
            self.tf_odom_robot_msg[i_cam_].transform.rotation = Quaternion(*ident_quat)

        self.tf_robot_laser_msg = TransformStamped()
        self.tf_robot_laser_msg.header.frame_id = "robot" 
        self.tf_robot_laser_msg.child_frame_id = "laser"  
        self.tf_robot_laser_msg.transform.rotation = Quaternion(*ident_quat)
        self.tf_robot_laser_msg.transform.translation.x = 0.3

        self.tf_world_map_init_msg = TransformStamped()
        self.tf_world_map_init_msg.header.frame_id = "world" 
        self.tf_world_map_init_msg.child_frame_id = "map"
        
        self.tf_map_odom_init_msg = TransformStamped()
        self.tf_map_odom_init_msg.header.frame_id = "map" 
        self.tf_map_odom_init_msg.child_frame_id = "odom"  
        self.tf_map_odom_init_msg.transform.rotation = Quaternion(*ident_quat)

        self.init_published = False

        self.quat_world_map_init = np.array([0,0,0,1])
        self.trans_world_map_init = np.array([0,0,0,0])
        self.quat_world_map_init_inv = np.array([0,0,0,1])
        self.trans_world_map_init_inv = np.array([0,0,0,0])

        self.quat_usbcam_world_inv = np.array([[0,0,0,1],[0,0,0,1]])
        self.trans_usbcam_world_inv = np.array([[0,0,0,0],[0,0,0,0]])

        self.start_time = rospy.Time.now().to_sec()
        self.map_start_time = -1




    def fiducial_transforms_cb(self, fid_msg: FiducialTransformArray):

        cam_ind = 0 if fid_msg.header.frame_id == "cam0" else 1 if fid_msg.header.frame_id == "cam1" else 2

        dt_start = (rospy.Time.now().to_sec()-self.start_time)
        dt_map = (rospy.Time.now().to_sec()-self.map_start_time)

        self.tf_world_map_init_msg.header.stamp = fid_msg.header.stamp
        self.tf_map_odom_init_msg.header.stamp = fid_msg.header.stamp

        self.tf_robot_laser_msg.header.stamp = fid_msg.header.stamp

        self.tf_odom_robot_msg[cam_ind].header.stamp = fid_msg.header.stamp
        self.tf_world_usbcam_msg[cam_ind].header.stamp = fid_msg.header.stamp

        for seq, fid_tf in enumerate(fid_msg.transforms):
            tf_quat = get_quat_arr_from_tf_msg(fid_tf)
            tf_trans = get_trans_arr_from_tf_msg(fid_tf)

            if fid_tf.fiducial_id == 42: # world -> usb_cam
                self.quat_usbcam_world_inv[cam_ind] = quaternion_inverse(tf_quat) 
                self.trans_usbcam_world_inv[cam_ind] = -rotate_vector_by_quat(tf_trans, self.quat_usbcam_world_inv[cam_ind]) 

                self.tf_world_usbcam_msg[cam_ind].transform.rotation = Quaternion(*self.quat_usbcam_world_inv[cam_ind])
                self.tf_world_usbcam_msg[cam_ind].transform.translation.x = self.trans_usbcam_world_inv[cam_ind][0]
                self.tf_world_usbcam_msg[cam_ind].transform.translation.y = self.trans_usbcam_world_inv[cam_ind][1]
                self.tf_world_usbcam_msg[cam_ind].transform.translation.z = self.trans_usbcam_world_inv[cam_ind][2]

            if fid_tf.fiducial_id == 46:   # world -> obot, world->map, map->odom, 
                # quat_world_robot = quaternion_multiply(self.quat_usbcam_world_inv[cam_ind], tf_quat)
                quat_world_robot = quaternion_multiply(
                                    quaternion_multiply(self.quat_usbcam_world_inv[cam_ind], tf_quat),
                                    flipz_quat) # doing Z flip
                quat_world_robot = quaternion_about_axis(euler_from_quaternion(quat_world_robot)[2], 
                                                         [0,0,1]) # using only rotation about Z

                trans_world_robot = rotate_vector_by_quat(tf_trans, self.quat_usbcam_world_inv[cam_ind]) + self.trans_usbcam_world_inv[cam_ind]
                trans_world_robot[2] = 0.0 # set Z to zero
            
                if not self.init_published:
                    self.init_published = True

                    self.quat_world_map_init = quat_world_robot.copy()
                    self.trans_world_map_init = trans_world_robot.copy()

                    self.tf_world_map_init_msg.transform.rotation = Quaternion(*self.quat_world_map_init)
                    self.tf_world_map_init_msg.transform.translation.x = self.trans_world_map_init[0]
                    self.tf_world_map_init_msg.transform.translation.y = self.trans_world_map_init[1]
                    self.tf_world_map_init_msg.transform.translation.z = self.trans_world_map_init[2]    

                    self.quat_world_map_init_inv = quaternion_inverse(self.quat_world_map_init) 
                    self.trans_world_map_init_inv = -rotate_vector_by_quat(self.trans_world_map_init, self.quat_world_map_init_inv) 
                    quat_odom_robot = ident_quat
                    trans_odom_robot = np.zeros(4)

                    rospy.loginfo("Sent static init TFs")
                else:

                    # quat_odom_robot = quaternion_multiply(self.quat_world_map_init_inv, quat_world_robot)
                    # trans_odom_robot = rotate_vector_by_quat(trans_world_robot, self.quat_world_map_init_inv) + self.trans_world_map_init_inv
                    quat_odom_robot = ident_quat
                    trans_odom_robot = np.zeros(4)

                self.tf_odom_robot_msg[cam_ind].transform.rotation = Quaternion(*quat_odom_robot) 
                self.tf_odom_robot_msg[cam_ind].transform.translation.x = trans_odom_robot[0]
                self.tf_odom_robot_msg[cam_ind].transform.translation.y = trans_odom_robot[1]
                self.tf_odom_robot_msg[cam_ind].transform.translation.z = trans_odom_robot[2]

                self.tf_broadcaster.sendTransform([
                                                    self.tf_odom_robot_msg[cam_ind],
                                                    self.tf_world_usbcam_msg[cam_ind], 
                                                    self.tf_robot_laser_msg, 
                                                    self.tf_world_map_init_msg, 
                                                   ])
                
                if (dt_start<20 and not dt_map>10) or dt_map>10:
                    self.tf_broadcaster.sendTransform(self.tf_map_odom_init_msg)

            



    def map_waiter_cb(self, map_msg: OccupancyGrid):
        self.map_start_time = rospy.Time.now().to_sec()





if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    reconstructor = SisyphStatePublisher(node_handler)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
