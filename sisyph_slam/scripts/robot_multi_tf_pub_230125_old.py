#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_conjugate as q_conj
from tf.transformations import quaternion_from_euler as euler_to_q
from tf.transformations import euler_from_quaternion as q_to_euler
from tf.transformations import quaternion_about_axis as q_axis
from tf.transformations import quaternion_inverse as inv_
from tf.transformations import quaternion_multiply as q_mult
# from sklearn.preprocessing import normalize

from matplotlib import pyplot as plt


def rot_by_q(_vect, _q):
    return q_mult(q_mult(_q, _vect), q_conj(_q))  #  _q^H(_p*_q)

def get_q_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.rotation.x, _msg.transform.rotation.y, _msg.transform.rotation.z, _msg.transform.rotation.w])
def get_p_arr_from_tf_msg(_msg):
    return np.array([_msg.transform.translation.x, _msg.transform.translation.y, _msg.transform.translation.z, 0.0])


correct_fid_q = euler_to_q(-3.1414,-3.1414,0)
ident_q = np.array([0, 0, 0, 1])
flipz_q = q_axis(3.1414, (0,0,1))



class SisyphStatePublisher:

    def __init__(self, nh):

        self.nh = nh

        self.fid_robot = 46
        self.fid_world = 42

        self.no_map_timeout = 10

        fid_listener0 = rospy.Subscriber("/aruco0/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)
        fid_listener1 = rospy.Subscriber("/aruco1/fiducial_transforms", FiducialTransformArray, self.fid_tf_cb)
        map_waiter = rospy.Subscriber("/map", OccupancyGrid, self.map_waiter_cb)

        self.tf_broadcaster0 = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster1 = tf2_ros.TransformBroadcaster()
        # static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.cams = ["cam0", "cam1"]

        self.fid0_tfs_msg = [TransformStamped(), TransformStamped()]

        self.tf_wc_msg = [TransformStamped(), TransformStamped()]
        self.tf_or_msg = [TransformStamped(), TransformStamped()]
        self.tf_wm_init_msg = [TransformStamped(), TransformStamped()]
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


        self.tf_rl_msg = TransformStamped() # STATIC
        self.tf_rl_msg.header.frame_id = "robot" 
        self.tf_rl_msg.child_frame_id = "laser"  
        self.tf_rl_msg.transform.rotation = Quaternion(*ident_q)
        self.tf_rl_msg.transform.translation.x = 0.2555
        self.tf_rl_msg.transform.translation.y = 0.078
        
        self.tf_mo_init_msg = TransformStamped() # STATIC
        self.tf_mo_init_msg.header.frame_id = "map" 
        self.tf_mo_init_msg.child_frame_id = "odom"  
        self.tf_mo_init_msg.transform.rotation = Quaternion(*ident_q)

        self.robot_found = np.array([False, False])
        self.world_found = np.array([False,False])

        self.q_wm_init = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_wm_init = [np.array([0,0,0,0]),np.array([0,0,0,0])]
        self.q_wm_init_inv = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_wm_init_inv = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.q_cw_inv_init = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_cw_inv_init = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.q_wr = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_wr = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.tf_last_time = [0.0, 0.0]

        self.q_wr_dt = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_wr_dt = [np.array([0,0,0,0]),np.array([0,0,0,0])]
        self.q_wr_dt_diff = np.array([0,0,0,1])
        self.p_wr_dt_diff = np.array([0,0,0,0])

        self.start_time = 0.0
        self.map_got_time = 0.0

        self.odom_pub_stopped = True

        self.pub_tfs_from = 0






    def process_tf_msg(self, transforms: list, cam_ind: int, tf_time: float):

        for fid_tf in transforms:
            tf_q = get_q_arr_from_tf_msg(fid_tf)
            tf_p = get_p_arr_from_tf_msg(fid_tf)

            if fid_tf.fiducial_id == self.fid_world: # world -> usb_cam
                if not self.world_found[cam_ind]:
                    if self.start_time==0.0: self.start_time = rospy.Time.now().to_sec()
                    rospy.loginfo(f"Found init cam{cam_ind} tf")
                    self.world_found[cam_ind] = True

                    self.q_cw_inv_init[cam_ind] = inv_(tf_q) 
                    self.p_cw_inv_init[cam_ind] = -rot_by_q(tf_p, self.q_cw_inv_init[cam_ind]) 

                    self.tf_wc_msg[cam_ind].transform.rotation = Quaternion(*self.q_cw_inv_init[cam_ind])
                    self.tf_wc_msg[cam_ind].transform.translation.x = self.p_cw_inv_init[cam_ind][0]
                    self.tf_wc_msg[cam_ind].transform.translation.y = self.p_cw_inv_init[cam_ind][1]
                    self.tf_wc_msg[cam_ind].transform.translation.z = self.p_cw_inv_init[cam_ind][2]

            if fid_tf.fiducial_id == self.fid_robot and self.world_found[cam_ind]:   # world -> obot, world->map, map->odom, 
                # q_wr = q_mult(self.q_cw_inv_init[cam_ind], tf_q)
                q_wr = q_mult(q_mult(self.q_cw_inv_init[cam_ind], tf_q), flipz_q) # doing Z flip
                q_wr = q_axis(q_to_euler(q_wr)[2],[0,0,1]) # using only rotation about Z

                p_wr = rot_by_q(tf_p, self.q_cw_inv_init[cam_ind]) + self.p_cw_inv_init[cam_ind]
                p_wr[2] = 0.0 # set Z to zero
            
                if not self.robot_found[cam_ind]:
                    self.robot_found[cam_ind] = True

                    self.q_wm_init[cam_ind]  = q_wr.copy()
                    self.p_wm_init[cam_ind]  = p_wr.copy()

                    self.tf_wm_init_msg[cam_ind].transform.rotation = Quaternion(*self.q_wm_init[cam_ind] )
                    self.tf_wm_init_msg[cam_ind].transform.translation.x = self.p_wm_init[cam_ind][0]
                    self.tf_wm_init_msg[cam_ind].transform.translation.y = self.p_wm_init[cam_ind][1]
                    self.tf_wm_init_msg[cam_ind].transform.translation.z = self.p_wm_init[cam_ind][2]    

                    self.q_wm_init_inv[cam_ind]  = inv_(self.q_wm_init[cam_ind]) 
                    self.p_wm_init_inv[cam_ind]  = -rot_by_q(self.p_wm_init[cam_ind], 
                                                             self.q_wm_init_inv[cam_ind]) 
                    self.q_wr[cam_ind] = ident_q
                    self.p_wr[cam_ind] = np.zeros(4)
                    q_or = ident_q
                    p_or = np.zeros(4)

                    # if 
                        # self.robot_found[cam_ind] = False
                    rospy.loginfo(f"Found init odom tf for cam{cam_ind}")

                else:
                    # self.q_wr_dt[cam_ind] = q_mult(inv_(self.q_wr[cam_ind]), q_wr)
                    # self.p_wr_dt[cam_ind] = rot_by_q(p_wr, inv_(self.q_wr[cam_ind])) - rot_by_q(self.p_wr[cam_ind], inv_(self.q_wr[cam_ind])) 
                    self.q_wr[cam_ind] = q_wr
                    self.p_wr[cam_ind] = p_wr

                    q_or = q_mult(self.q_wm_init_inv[cam_ind], self.q_wr[cam_ind])
                    p_or = rot_by_q(self.p_wr[cam_ind], self.q_wm_init_inv[cam_ind]) + self.p_wm_init_inv[cam_ind] 

                self.tf_or_msg[cam_ind].transform.rotation = Quaternion(*q_or) 
                self.tf_or_msg[cam_ind].transform.translation.x = p_or[0]
                self.tf_or_msg[cam_ind].transform.translation.y = p_or[1]
                self.tf_or_msg[cam_ind].transform.translation.z = p_or[2]

                # print("quat:",[f"{q0-q1:.4f}" for q0, q1 in zip(self.q_wr_dt[0],self.q_wr_dt[1])])
                # print("tran:",[f"{p0-p1:.4f}" for p0, p1 in zip(self.p_wr_dt[0],self.p_wr_dt[1])])
                # print(f"cam0: {np.linalg.norm(self.p_wr_dt[0],1):.4f}, cam1: {np.linalg.norm(self.p_wr_dt[1],1):.4f}")




    def choose_cam(self, tf_time):
        q0_diff= q_to_euler(self.q_wr_dt[0])[2]
        q1_diff = q_to_euler(self.q_wr_dt[1])[2]

        q0_cond = q0_diff<(np.pi/15)
        q1_cond = q1_diff<(np.pi/15)
        q01_cond = q0_diff<q1_diff

        p0_cond = np.max(self.p_wr_dt[0])<0.02
        p1_cond = np.max(self.p_wr_dt[1])<0.02
        p01_cond = np.max(self.p_wr_dt[0])<np.max(self.p_wr_dt[1])

        dt0_cond = (self.tf_last_time[0]-tf_time)<0.1
        dt1_cond = (self.tf_last_time[1]-tf_time)<0.1

        new_pub_tfs_from = 0
        if dt0_cond: 
            if q0_cond and p0_cond: new_pub_tfs_from = 0
            else:
                if q1_cond and p1_cond: new_pub_tfs_from = 1
                else:
                    if q01_cond: new_pub_tfs_from = 0
                    else: 
                        if p01_cond: new_pub_tfs_from = 0
                        else: new_pub_tfs_from = 1
        else:
            if q1_cond and p1_cond: new_pub_tfs_from = 1
            else:
                if q0_cond and p0_cond: new_pub_tfs_from = 0
                else:
                    if not q01_cond: new_pub_tfs_from = 1
                    else: 
                        if not p01_cond: new_pub_tfs_from = 1
                        else: new_pub_tfs_from = 0     

        if new_pub_tfs_from!=self.pub_tfs_from:
            print(f"{self.pub_tfs_from}-->{new_pub_tfs_from},\t d0:{dt0_cond:1},\t d1:{dt1_cond:1},\t q0:{q0_cond:1},\t q1:{q1_cond:1},\t p0:{p0_cond:1},\t p1:{p1_cond:1},\t q01:{q01_cond:1},\t p01: {p01_cond:1}")
        self.pub_tfs_from = new_pub_tfs_from



    def fid_tf_cb(self, fid_msg: FiducialTransformArray):

        tf_time = fid_msg.header.stamp.to_sec()

        tfs_to_pub = []

        cam_ind = int(fid_msg.header.frame_id[-1])
        # if self.world_found.all() and cam_ind==0 and self.robot_found.all(): return

        if len(fid_msg.transforms)>0:

            self.process_tf_msg(fid_msg.transforms, cam_ind, tf_time)

            if self.world_found[cam_ind]:

                if self.world_found.all(): self.choose_cam(tf_time) 
                else: self.pub_tfs_from = cam_ind

                self.tf_wc_msg[cam_ind].header.stamp = fid_msg.header.stamp        
                tfs_to_pub.append(self.tf_wc_msg[cam_ind])                
                
                if cam_ind==self.pub_tfs_from:

                    self.tf_or_msg[cam_ind].header.stamp = fid_msg.header.stamp
                    tfs_to_pub.append(self.tf_or_msg[cam_ind])

                    self.tf_wm_init_msg[cam_ind].header.stamp = fid_msg.header.stamp
                    tfs_to_pub.append(self.tf_wm_init_msg[cam_ind])

                    self.tf_rl_msg.header.stamp = fid_msg.header.stamp  
                    tfs_to_pub.append(self.tf_rl_msg)

                    dt_start = (rospy.Time.now().to_sec()-self.start_time)
                    dt_map = (rospy.Time.now().to_sec()-self.map_got_time)
                    maps_too_old = True #dt_map>self.no_map_timeout
                    starting = dt_start<self.no_map_timeout
                    odom_pub_cond = starting or maps_too_old

                    if odom_pub_cond:
                        self.tf_mo_init_msg.header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_mo_init_msg)
                        
                        if self.odom_pub_stopped:
                            self.odom_pub_stopped = False
                            rospy.loginfo("Started sending map->odom frame")
                    else:
                        if not self.odom_pub_stopped:
                            self.odom_pub_stopped = True
                            rospy.loginfo("Map is building! Stopped sending map->odom frame")

                self.tf_broadcaster0.sendTransform(tfs_to_pub)
            self.tf_last_time[cam_ind] = tf_time


    def map_waiter_cb(self, map_msg: OccupancyGrid):
        self.map_got_time = rospy.Time.now().to_sec()





if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    robot_tf_publisher = SisyphStatePublisher(node_handler)

    try:
        rospy.spin()
        # while not rospy.core.is_shutdown():
    except rospy.ROSInterruptException:
        pass

