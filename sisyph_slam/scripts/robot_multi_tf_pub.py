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

        self.fid_robot = 42
        self.fid_world = 46

        self.no_map_timeout = 10

        fid_listener0 = rospy.Subscriber("/aruco0/fiducial_transforms", FiducialTransformArray, self.fid_tf0_cb)
        fid_listener1 = rospy.Subscriber("/aruco1/fiducial_transforms", FiducialTransformArray, self.fid_tf1_cb)
        map_waiter = rospy.Subscriber("/map", OccupancyGrid, self.map_waiter_cb)

        self.tf_broadcaster0 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster1 = tf2_ros.TransformBroadcaster()
        # static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.cams = ["cam0", "cam1"]

        self.tf_world_usbcam_msg = [TransformStamped(), TransformStamped()]
        self.tf_odom_robot_msg = [TransformStamped(), TransformStamped()]
        self.tf_world_map_init_msg = [TransformStamped(), TransformStamped()]
        for i_cam_, cam_ in enumerate(self.cams):
            self.tf_world_usbcam_msg[i_cam_].header.frame_id = "world" 
            self.tf_world_usbcam_msg[i_cam_].child_frame_id = cam_
            self.tf_world_usbcam_msg[i_cam_].transform.rotation = Quaternion(*ident_quat)
 
            self.tf_odom_robot_msg[i_cam_].header.frame_id = "odom" 
            self.tf_odom_robot_msg[i_cam_].child_frame_id = "robot" 
            self.tf_odom_robot_msg[i_cam_].transform.rotation = Quaternion(*ident_quat)

            self.tf_world_map_init_msg[i_cam_].header.frame_id = "world" 
            self.tf_world_map_init_msg[i_cam_].child_frame_id = "map"
            self.tf_world_map_init_msg[i_cam_].transform.rotation = Quaternion(*ident_quat)


        self.tf_robot_laser_msg = TransformStamped() # STATIC
        self.tf_robot_laser_msg.header.frame_id = "robot" 
        self.tf_robot_laser_msg.child_frame_id = "laser"  
        self.tf_robot_laser_msg.transform.rotation = Quaternion(*ident_quat)
        self.tf_robot_laser_msg.transform.translation.x = 0.199
        self.tf_robot_laser_msg.transform.translation.y = 0.073
        
        self.tf_map_odom_init_msg = TransformStamped() # STATIC
        self.tf_map_odom_init_msg.header.frame_id = "map" 
        self.tf_map_odom_init_msg.child_frame_id = "odom"  
        self.tf_map_odom_init_msg.transform.rotation = Quaternion(*ident_quat)

        self.init_published = [False, False]
        self.world_found = [False,False]

        self.quat_world_map_init = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.trans_world_map_init = [np.array([0,0,0,0]),np.array([0,0,0,0])]
        self.quat_world_map_init_inv = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.trans_world_map_init_inv = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.quat_usbcam_world_inv = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.trans_usbcam_world_inv = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.start_time = rospy.Time.now().to_sec()
        self.map_got_time = rospy.Time.now().to_sec()-self.no_map_timeout-1

        self.odom_pub_stopped = True





    def process_tf_msg(self, fid_msg: FiducialTransformArray, cam_ind: int):

        self.tf_map_odom_init_msg.header.stamp = fid_msg.header.stamp
        self.tf_robot_laser_msg.header.stamp = fid_msg.header.stamp

        self.tf_world_map_init_msg[cam_ind].header.stamp = fid_msg.header.stamp
        self.tf_odom_robot_msg[cam_ind].header.stamp = fid_msg.header.stamp
        self.tf_world_usbcam_msg[cam_ind].header.stamp = fid_msg.header.stamp

        for seq, fid_tf in enumerate(fid_msg.transforms):
            tf_quat = get_quat_arr_from_tf_msg(fid_tf)
            tf_trans = get_trans_arr_from_tf_msg(fid_tf)

            if fid_tf.fiducial_id == self.fid_world: # world -> usb_cam
                if not self.world_found[cam_ind]:
                    self.world_found[cam_ind] = True

                    self.quat_usbcam_world_inv[cam_ind] = quaternion_inverse(tf_quat) 
                    self.trans_usbcam_world_inv[cam_ind] = -rotate_vector_by_quat(tf_trans, self.quat_usbcam_world_inv[cam_ind]) 

                    self.tf_world_usbcam_msg[cam_ind].transform.rotation = Quaternion(*self.quat_usbcam_world_inv[cam_ind])
                    self.tf_world_usbcam_msg[cam_ind].transform.translation.x = self.trans_usbcam_world_inv[cam_ind][0]
                    self.tf_world_usbcam_msg[cam_ind].transform.translation.y = self.trans_usbcam_world_inv[cam_ind][1]
                    self.tf_world_usbcam_msg[cam_ind].transform.translation.z = self.trans_usbcam_world_inv[cam_ind][2]

            if fid_tf.fiducial_id == self.fid_robot and self.world_found[cam_ind]:   # world -> obot, world->map, map->odom, 
                # quat_world_robot = quaternion_multiply(self.quat_usbcam_world_inv[cam_ind], tf_quat)
                quat_world_robot = quaternion_multiply(
                                    quaternion_multiply(self.quat_usbcam_world_inv[cam_ind], tf_quat),
                                    flipz_quat) # doing Z flip
                quat_world_robot = quaternion_about_axis(euler_from_quaternion(quat_world_robot)[2], 
                                                         [0,0,1]) # using only rotation about Z

                trans_world_robot = rotate_vector_by_quat(tf_trans, self.quat_usbcam_world_inv[cam_ind]) + self.trans_usbcam_world_inv[cam_ind]
                trans_world_robot[2] = 0.0 # set Z to zero
            
                if not self.init_published[cam_ind]:
                    self.init_published[cam_ind] = True

                    self.quat_world_map_init[cam_ind]  = quat_world_robot.copy()
                    self.trans_world_map_init[cam_ind]  = trans_world_robot.copy()

                    self.tf_world_map_init_msg[cam_ind].transform.rotation = Quaternion(*self.quat_world_map_init[cam_ind] )
                    self.tf_world_map_init_msg[cam_ind].transform.translation.x = self.trans_world_map_init[cam_ind][0]
                    self.tf_world_map_init_msg[cam_ind].transform.translation.y = self.trans_world_map_init[cam_ind][1]
                    self.tf_world_map_init_msg[cam_ind].transform.translation.z = self.trans_world_map_init[cam_ind][2]    

                    self.quat_world_map_init_inv[cam_ind]  = quaternion_inverse(self.quat_world_map_init[cam_ind] ) 
                    self.trans_world_map_init_inv[cam_ind]  = -rotate_vector_by_quat(self.trans_world_map_init[cam_ind] , self.quat_world_map_init_inv[cam_ind]) 
                    quat_odom_robot = ident_quat
                    trans_odom_robot = np.zeros(4)

                    rospy.loginfo(f"Sent static init TFs: cam{cam_ind}")
                else:

                    quat_odom_robot = quaternion_multiply(self.quat_world_map_init_inv[cam_ind], quat_world_robot)
                    trans_odom_robot = rotate_vector_by_quat(trans_world_robot, self.quat_world_map_init_inv[cam_ind]) + self.trans_world_map_init_inv[cam_ind] 

                self.tf_odom_robot_msg[cam_ind].transform.rotation = Quaternion(*quat_odom_robot) 
                self.tf_odom_robot_msg[cam_ind].transform.translation.x = trans_odom_robot[0]
                self.tf_odom_robot_msg[cam_ind].transform.translation.y = trans_odom_robot[1]
                self.tf_odom_robot_msg[cam_ind].transform.translation.z = trans_odom_robot[2]


            





    def fid_tf0_cb(self, fid_msg: FiducialTransformArray):

        cam_ind = 0

        if len(fid_msg.transforms)>0 and fid_msg.header.frame_id == f"cam{cam_ind}":

            dt_start = (rospy.Time.now().to_sec()-self.start_time)
            dt_map = (rospy.Time.now().to_sec()-self.map_got_time)

            self.process_tf_msg(fid_msg, cam_ind)

            if self.world_found[cam_ind]:
                self.tf_broadcaster0.sendTransform([
                                                    self.tf_odom_robot_msg[cam_ind],
                                                    self.tf_world_usbcam_msg[cam_ind], 
                                                    self.tf_world_map_init_msg[cam_ind], 
                                                    self.tf_robot_laser_msg, 
                                                    ])
                
                maps_too_old = dt_map>self.no_map_timeout
                starting = dt_start<self.no_map_timeout
                odom_pub_cond = starting or maps_too_old

                if odom_pub_cond:
                    self.tf_broadcaster0.sendTransform(self.tf_map_odom_init_msg)
                    
                    if self.odom_pub_stopped:
                        self.odom_pub_stopped = False
                        rospy.loginfo("Started sending map->odom frame")
                else:
                    if not self.odom_pub_stopped:
                        self.odom_pub_stopped = True
                        rospy.loginfo("Map is building! Stopped sending map->odom frame")






    def fid_tf1_cb(self, fid_msg: FiducialTransformArray):

        cam_ind = 1

        if len(fid_msg.transforms)>0 and fid_msg.header.frame_id == f"cam{cam_ind}":

            self.process_tf_msg(fid_msg, cam_ind)

            if self.world_found[cam_ind]:
                self.tf_broadcaster1.sendTransform([
                                                    self.tf_odom_robot_msg[cam_ind],
                                                    self.tf_world_map_init_msg[cam_ind],
                                                    self.tf_world_usbcam_msg[cam_ind], 
                                                    ])
            
            # dt_start = (rospy.Time.now().to_sec()-self.start_time)
            # dt_map = (rospy.Time.now().to_sec()-self.map_got_time)

            # self.process_tf_msg(fid_msg, cam_ind)

            # if self.world_found[cam_ind]:
            #     self.tf_broadcaster0.sendTransform([
            #                                         self.tf_odom_robot_msg[cam_ind],
            #                                         self.tf_world_usbcam_msg[cam_ind], 
            #                                         self.tf_world_map_init_msg[cam_ind], 
            #                                         self.tf_robot_laser_msg, 
            #                                         ])
        
            #     if (dt_start<20 and not dt_map>10) or dt_map>10:
            #         self.tf_broadcaster0.sendTransform(self.tf_map_odom_init_msg)





    def map_waiter_cb(self, map_msg: OccupancyGrid):
        self.map_got_time = rospy.Time.now().to_sec()





if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    reconstructor = SisyphStatePublisher(node_handler)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
