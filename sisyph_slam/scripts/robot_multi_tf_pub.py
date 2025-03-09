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
flipz_q = q_axis(3.1414, (0,0,1))



class SisyphStatePublisher:

    def __init__(self, nh, fid_robot: int, fid_world: int, laser_offset_xy: list):

        self.nh = nh

        self.fid_robot = fid_robot
        self.fid_world = fid_world

        self.no_map_timeout = 10

        fid_listener0 = rospy.Subscriber("/aruco0/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)
        fid_listener1 = rospy.Subscriber("/aruco1/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)
        map_waiter = rospy.Subscriber("/map", OccupancyGrid, self.map_waiter_cb)

        self.tf_broadcaster0 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster1 = tf2_ros.TransformBroadcaster()
        # static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.cams = ["cam0", "cam1"]
        self.prior_cam = 0
        self.second_cam = int(abs(1-self.prior_cam))
        
        self.fid0_tfs_msg = [TransformStamped(), TransformStamped()]

        self.tf_wc_msg = [TransformStamped(), TransformStamped()]
        self.tf_wm_init_msg = [TransformStamped(), TransformStamped()]        
        self.tf_mo_init_msg = [TransformStamped(), TransformStamped()]        
        self.tf_or_msg = [TransformStamped(), TransformStamped()]
        for i_cam_, cam_ in enumerate(self.cams):
            self.tf_wc_msg[i_cam_].header.frame_id = "world" 
            self.tf_wc_msg[i_cam_].child_frame_id = cam_
            self.tf_wc_msg[i_cam_].transform.rotation = Quaternion(*ident_q)
 
            self.tf_or_msg[i_cam_].header.frame_id = "odom" 
            self.tf_or_msg[i_cam_].child_frame_id = "robot" 
            self.tf_or_msg[i_cam_].transform.rotation = Quaternion(*ident_q)

            self.tf_wm_init_msg[i_cam_].header.frame_id = "world" 
            self.tf_wm_init_msg[i_cam_].child_frame_id = "map"
            self.tf_wm_init_msg[i_cam_].transform.rotation = Quaternion(*ident_q)

            self.tf_mo_init_msg[i_cam_].header.frame_id = "map" 
            self.tf_mo_init_msg[i_cam_].child_frame_id = "odom"  
            self.tf_mo_init_msg[i_cam_].transform.rotation = Quaternion(*ident_q)            

        self.tf_wm_init_msg[1].child_frame_id = "map1"
        self.tf_or_msg[1].child_frame_id = "robot1" 
        self.tf_mo_init_msg[1].header.frame_id = "map1" 
        self.tf_mo_init_msg[1].child_frame_id = "odom1" 

        self.tf_rl_msg = TransformStamped() # STATIC
        self.tf_rl_msg.header.frame_id = "robot" 
        self.tf_rl_msg.child_frame_id = "laser"  
        self.tf_rl_msg.transform.rotation = Quaternion(*ident_q)
        self.tf_rl_msg.transform.translation.x = laser_offset_xy[0]
        self.tf_rl_msg.transform.translation.y = laser_offset_xy[1]
        

        
        self.pos_track = Marker()
        self.pos_track.header.frame_id = "odom"
        self.pos_track.ns = "/pos_track"
        self.pos_track.id = 0
        self.pos_track.lifetime = rospy.Duration(30)
        self.pos_track.type = Marker.SPHERE
        self.pos_track.action = Marker.ADD
        self.pos_track.scale.x = 0.02
        self.pos_track.scale.y = 0.02
        self.pos_track.scale.z = 0.02
        self.pos_track.color.r = 200
        self.pos_track.color.g = 200
        self.pos_track.color.b = 0        
        self.pos_track.color.a = 0.9

        self.max_pos_track = 500
        self.pos_track_pub = rospy.Publisher("/pos_track", Marker, queue_size=30)
        
        self.robot_found = np.array([False,False])
        self.world_found = np.array([False,False])

        self.qp_cw_inv_init = [(np.array([0,0,0,1]),np.array([0,0,0,0])),
                               (np.array([0,0,0,1]),np.array([0,0,0,0]))]
        self.qp_wm_init = [(np.array([0,0,0,1]),np.array([0,0,0,0])),
                           (np.array([0,0,0,1]),np.array([0,0,0,0]))]
        self.qp_wm_init_inv = [(np.array([0,0,0,1]),np.array([0,0,0,0])),
                               (np.array([0,0,0,1]),np.array([0,0,0,0]))]       
        self.qp_or = [(np.array([0,0,0,1]),np.array([0,0,0,0])),
                      (np.array([0,0,0,1]),np.array([0,0,0,0]))]

        self.pred_qp = [(np.array([0,0,0,1]),np.array([0,0,0,0])),
                        (np.array([0,0,0,1]),np.array([0,0,0,0]))]
        self.pred_ok = np.array([False,False])
        self.lost_cam = np.array([True,True])

        self.tf_last_time = [0.0, 0.0]

        self.start_time = 0.0
        self.map_got_time = 0.0

        self.odom_pub_stopped = True

        self.pub_tfs_from = 0






    def process_tf_msg(self, transforms: list, cam_: int, tf_time: float):

        for fid_tf in transforms:
            tf_qp = qp_from_tf_msg(fid_tf)

            if fid_tf.fiducial_id == self.fid_world: # world -> usb_cam
                if not self.world_found[cam_]:
                    if self.start_time==0.0: self.start_time = rospy.Time.now().to_sec()

                    rospy.loginfo(f"Found world for cam{cam_}")
                    self.world_found[cam_] = True

                    self.qp_cw_inv_init[cam_] = qp_inv(tf_qp) # T_cw_inv = T_wc
                    self.tf_wc_msg[cam_] = qp_to_tf_msg(self.qp_cw_inv_init[cam_], self.tf_wc_msg[cam_])                                                           
            
        if fid_tf.fiducial_id == self.fid_robot and self.world_found[cam_]:   # world -> robot, world->map, map->odom, 
                q_wr, p_wr = qp_mult(self.qp_cw_inv_init[cam_], tf_qp)
                q_wr = q_mult(q_wr, flipz_q)
                q_wr = q_axis(q_to_euler(q_wr)[2],[0,0,1])  
                p_wr[2] = 0.0 
                qp_wr = (q_wr, p_wr) # T_wr

                if not self.robot_found[cam_]:
                    self.robot_found[cam_] = True
                    rospy.loginfo(f"Found robot for cam{cam_}")                

                    self.qp_wm_init[cam_]  = qp_wr
                    self.tf_wm_init_msg[cam_] = qp_to_tf_msg(self.qp_wm_init[cam_], self.tf_wm_init_msg[cam_])  

                    self.qp_wm_init_inv[cam_] = qp_inv(self.qp_wm_init[cam_])

                else:

                    qp_or = qp_mult(self.qp_wm_init_inv[cam_], qp_wr)

                    if self.robot_found[self.prior_cam] and self.world_found[self.prior_cam]:  

                        q_pred_err, p_pred_err = qp_relative(self.pred_qp[cam_], qp_or)
                        angle_err, pos_err = abs(q_to_euler(q_pred_err)[2]), np.linalg.norm(p_pred_err[:3])
                        self.pred_ok[cam_] = angle_err<0.01 and pos_err<0.05
                        self.lost_cam[cam_] = (tf_time-self.tf_last_time[0])>0.8

                        if cam_== self.prior_cam: 
                            self.pub_tfs_from = self.prior_cam if self.pred_ok[cam_] and not self.lost_cam[cam_] else -1

                        # if cam_==1: print(f"angle_err={angle_err:.5f}, pos_err={pos_err:.5f}")

                    #     if self.pred_ok[self.prior_cam] and not self.lost_cam[self.prior_cam]:
                    #         self.pub_tfs_from = self.prior_cam
                    #     else:
                    #         if self.pred_ok[self.second_cam] and not self.lost_cam[self.second_cam]:
                    #             self.pub_tfs_from = self.second_cam
                    #         else:
                    #             self.pub_tfs_from = -1

                    #     if self.lost_cam.any(): print(f"lost: cam0:{self.lost_cam[0]}, cam1:{self.lost_cam[1]}")

                    pred_step = qp_relative(self.qp_or[cam_], qp_or)
                    self.pred_qp[cam_] = qp_mult(qp_or, pred_step)

                    self.qp_or[cam_] = qp_or

                self.tf_or_msg[cam_] = qp_to_tf_msg(self.qp_or[cam_], self.tf_or_msg[cam_])





    def fid_tf_cb(self, fid_msg: FiducialTransformArray):

        tf_time = fid_msg.header.stamp.to_sec()

        tfs_to_pub = []

        cam_ = int(fid_msg.header.frame_id[-1])
        # if self.world_found.all() and cam_==0 and self.robot_found.all(): return

        if len(fid_msg.transforms)>0:

            self.process_tf_msg(fid_msg.transforms, cam_, tf_time)

            if self.world_found[cam_]:

                self.tf_wc_msg[cam_].header.stamp = fid_msg.header.stamp        
                tfs_to_pub.append(self.tf_wc_msg[cam_])      

                odom_pub_cond = self.map_got_time-tf_time

                if not self.pub_tfs_from==-1:
                    if cam_==self.pub_tfs_from:

                        self.tf_wm_init_msg[cam_].header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_wm_init_msg[cam_])                                          
                        self.tf_or_msg[cam_].header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_or_msg[cam_])

                        self.tf_rl_msg.header.stamp = fid_msg.header.stamp  
                        tfs_to_pub.append(self.tf_rl_msg)

                        maps_too_old = (rospy.Time.now().to_sec()-self.map_got_time)>self.no_map_timeout
                        starting = (rospy.Time.now().to_sec()-self.start_time)<self.no_map_timeout
                        odom_pub_cond = starting or maps_too_old
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
    robot_tf_publisher = SisyphStatePublisher(node_handler, 42, 46, [0.2555, 0.078])

    try:
        rospy.spin()
        # while not rospy.core.is_shutdown():
    except rospy.ROSInterruptException:
        pass

