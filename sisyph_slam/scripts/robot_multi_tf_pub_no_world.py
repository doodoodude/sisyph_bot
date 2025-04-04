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



correct_fid_q = euler_to_q(-3.1414,-3.1414,0)
ident_q = np.array([0, 0, 0, 1])
zero_p = np.array([0, 0, 0, 0])
flipz_q = q_axis(3.1414, (0,0,1))



class SisyphStatePublisher:

    def __init__(self, nh, fid_robot: int, fid_world: int, fid_robot_offset_xy: list):

        self.nh = nh

        self.fid_robot = fid_robot
        self.fid_world = fid_world

        self.no_map_timeout = 12
        self.use_own_odom = True

        fid_listener0 = rospy.Subscriber("/aruco0/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)
        # fid_listener1 = rospy.Subscriber("/aruco1/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)
        map_waiter = rospy.Subscriber("/map", OccupancyGrid, self.map_waiter_cb)

        self.tf_broadcaster0 = tf2_ros.TransformBroadcaster()

        self.cams = ["cam0", "cam1"]
        self.prior_cam = 0
        self.second_cam = int(abs(1-self.prior_cam))

        self.n_init_control_frames = 30
        
        self.fid0_tfs_msg = [TransformStamped(), TransformStamped()]

        self.tf_mc_msg = [TransformStamped(), TransformStamped()]       
        self.tf_mo_init_msg = [TransformStamped(), TransformStamped()]        
        self.tf_or_msg = [TransformStamped(), TransformStamped()]
        for i_cam_, cam_ in enumerate(self.cams):
            self.tf_mc_msg[i_cam_].header.frame_id = "map" 
            self.tf_mc_msg[i_cam_].child_frame_id = cam_
            self.tf_mc_msg[i_cam_].transform.rotation = Quaternion(*ident_q)
 
            self.tf_or_msg[i_cam_].header.frame_id = "odom" 
            self.tf_or_msg[i_cam_].child_frame_id = "robot0" 
            self.tf_or_msg[i_cam_].transform.rotation = Quaternion(*ident_q)

            self.tf_mo_init_msg[i_cam_].header.frame_id = "map" 
            self.tf_mo_init_msg[i_cam_].child_frame_id = "odom"  
            self.tf_mo_init_msg[i_cam_].transform.rotation = Quaternion(*ident_q)            

        self.tf_or_msg[1].child_frame_id = "robot1" 
        self.tf_mo_init_msg[1].header.frame_id = "map1" 
        self.tf_mo_init_msg[1].child_frame_id = "odom1" 
        
        self.pos_track = Marker()
        self.pos_track.header.frame_id = "odom"
        self.pos_track.ns = "/pos_track"
        self.pos_track.id = 0
        self.pos_track.lifetime = rospy.Duration(30)
        self.pos_track.type = Marker.ARROW
        self.pos_track.action = Marker.ADD

        self.pos_track.scale.x = 0.02
        self.pos_track.scale.y = 0.01
        self.pos_track.scale.z = 0.01
        self.pos_track.color.r = 200
        self.pos_track.color.g = 200
        self.pos_track.color.b = 0        
        self.pos_track.color.a = 0.9

        self.max_pos_track = 500
        self.pos_track_pub = rospy.Publisher("/sisyph/pos_track", Marker, queue_size=30)

        self.world_found = np.array([False,False])

        self.fid_offset = np.array([fid_robot_offset_xy[0],fid_robot_offset_xy[1],0,0])

        self.qp_cm_inv_init = [(ident_q,zero_p),
                               (ident_q,zero_p)]    

        self.lost_cam = np.array([True,True])

        self.tf_last_time = [0.0, 0.0]

        self.start_time = 0.0
        self.map_got_time = 0.0

        self.odom_pub_stopped = True

        self.pub_tfs_from = self.prior_cam






    def process_tf_msg(self, transforms: list, cam_: int, tf_time: float):

        for fid_tf in transforms:
            qp_fid = qp_mult(qp_from_tf_msg(fid_tf), (flipz_q, zero_p))
                                                  
            if fid_tf.fiducial_id == self.fid_robot:   
                if not self.world_found[cam_]:

                    self.n_init_control_frames -= 1
                    if self.n_init_control_frames==0:
                        if self.start_time==0.0: self.start_time = rospy.Time.now().to_sec()

                        rospy.loginfo(f"Found world for cam{cam_}")
                        self.world_found[cam_] = True

                        self.qp_cm_inv_init[cam_] = qp_inv(qp_fid) # T_cm_inv = T_mc
                        self.tf_mc_msg[cam_] = qp_to_tf_msg(self.qp_cm_inv_init[cam_], self.tf_mc_msg[cam_])    
                
                else:
                    q_or, p_or = qp_mult(self.qp_cm_inv_init[cam_], qp_fid) 
                    q_or = q_axis(q_to_euler(q_or)[2],[0,0,1])  
                    qp_or = (q_or, p_or*np.array([1,1,0,0])) # T_wr
                    self.tf_or_msg[cam_] = qp_to_tf_msg(qp_or, self.tf_or_msg[cam_])


    def fid_tf_cb(self, fid_msg: FiducialTransformArray):

        tf_time = fid_msg.header.stamp.to_sec()

        tfs_to_pub = []

        cam_ = int(fid_msg.header.frame_id[-1])

        if len(fid_msg.transforms)>0:

            self.process_tf_msg(fid_msg.transforms, cam_, tf_time)

            if self.world_found[cam_]:

                self.tf_mc_msg[cam_].header.stamp = fid_msg.header.stamp        
                tfs_to_pub.append(self.tf_mc_msg[cam_])      

                odom_pub_cond = self.map_got_time-tf_time

                if not self.pub_tfs_from==-1:
                    if cam_==self.pub_tfs_from:
                                         
                        self.tf_or_msg[cam_].header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_or_msg[cam_])

                        maps_too_old = (rospy.Time.now().to_sec()-self.map_got_time)>self.no_map_timeout
                        starting = (rospy.Time.now().to_sec()-self.start_time)<self.no_map_timeout
                        odom_pub_cond = starting or maps_too_old or self.use_own_odom
                        if odom_pub_cond:
                            self.tf_mo_init_msg[cam_].header.stamp = fid_msg.header.stamp
                            tfs_to_pub.append(self.tf_mo_init_msg[cam_])
                            
                            if self.odom_pub_stopped:
                                self.odom_pub_stopped = False
                                rospy.loginfo("Started sending map->odom frame")
                        else:
                            if not self.odom_pub_stopped:
                                self.odom_pub_stopped = True
                                rospy.loginfo("Map is building! Stopped sending map->odom frame")                                                

                        self.update_pos_track(self.tf_or_msg[cam_].transform)

                self.tf_broadcaster0.sendTransform(tfs_to_pub)
            self.tf_last_time[cam_] = tf_time



    def update_pos_track(self, transform):
        pos_param = self.pos_track.id/self.max_pos_track  
        color_ = np.abs(np.array([np.cos(pos_param*np.pi), np.sin(pos_param*np.pi*2), pos_param]))
        color_ /= np.linalg.norm(color_)
        self.pos_track.color.r = color_[0]
        self.pos_track.color.g = color_[1]
        self.pos_track.color.b = color_[2]
        self.pos_track.pose = tf_to_pose(self.pos_track.pose, transform)
        self.pos_track.id += 1
        if self.pos_track.id > self.max_pos_track: 
            self.pos_track.action = Marker.DELETEALL
            self.pos_track.id = -1
        self.pos_track_pub.publish(self.pos_track)
        self.pos_track.action = Marker.ADD        




    def map_waiter_cb(self, map_msg: OccupancyGrid):
        self.map_got_time = rospy.Time.now().to_sec()





if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    tf_pubber = SisyphStatePublisher(node_handler, 42, 46, [0.030, 0.071]) 

    try:
        rospy.spin()
        # while not rospy.core.is_shutdown():
        #     tf_pubber.tf_rl_msg.header.stamp = rospy.Time.now()
        #     tf_pubber.tf_broadcaster1.sendTransform(tf_pubber.tf_rl_msg)
        #     rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass

