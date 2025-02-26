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
    return q_mult(q_mult(_q, _vect), q_conj(_q))

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

        self.init_published = [False, False]
        self.world_found = [False,False]

        self.q_wm_init = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_wm_init = [np.array([0,0,0,0]),np.array([0,0,0,0])]
        self.q_wm_init_inv = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_wm_init_inv = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.q_cw_inv = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_cw_inv = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.q_or = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_or = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.tf_time_delta = [0.0, 0.0]

        self.q_or_dt = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_or_dt = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.jumped = [False, False]

        self.p_dt_norms = np.zeros(1500)
        self.p_dt_time_deltas = np.zeros(1500)
        self.p_dt_norms_ptr = 0

        self.start_time = rospy.Time.now().to_sec()
        self.map_got_time = rospy.Time.now().to_sec()-self.no_map_timeout-1

        self.odom_pub_stopped = True








    def process_tf_msg(self, fid_msg: FiducialTransformArray, cam_ind: int):

        for fid_tf in fid_msg.transforms:
            tf_q = get_q_arr_from_tf_msg(fid_tf)
            tf_p = get_p_arr_from_tf_msg(fid_tf)

            if fid_tf.fiducial_id == self.fid_world: # world -> usb_cam
                if not self.world_found[cam_ind]:

                    self.start_time = rospy.Time.now().to_sec()

                    rospy.loginfo(f"Found init cam{cam_ind} tf")
                    self.world_found[cam_ind] = True

                    self.q_cw_inv[cam_ind] = inv_(tf_q) 
                    self.p_cw_inv[cam_ind] = -rot_by_q(tf_p, self.q_cw_inv[cam_ind]) 

                    self.tf_wc_msg[cam_ind].transform.rotation = Quaternion(*self.q_cw_inv[cam_ind])
                    self.tf_wc_msg[cam_ind].transform.translation.x = self.p_cw_inv[cam_ind][0]
                    self.tf_wc_msg[cam_ind].transform.translation.y = self.p_cw_inv[cam_ind][1]
                    self.tf_wc_msg[cam_ind].transform.translation.z = self.p_cw_inv[cam_ind][2]

            if fid_tf.fiducial_id == self.fid_robot and self.world_found[cam_ind]:   # world -> obot, world->map, map->odom, 
                # q_wr = q_mult(self.q_cw_inv[cam_ind], tf_q)
                q_wr = q_mult(q_mult(self.q_cw_inv[cam_ind], tf_q), flipz_q) # doing Z flip
                q_wr = q_axis(q_to_euler(q_wr)[2],[0,0,1]) # using only rotation about Z

                p_wr = rot_by_q(tf_p, self.q_cw_inv[cam_ind]) + self.p_cw_inv[cam_ind]
                p_wr[2] = 0.0 # set Z to zero
            
                if not self.init_published[cam_ind]:
                    self.init_published[cam_ind] = True

                    self.q_wm_init[cam_ind]  = q_wr.copy()
                    self.p_wm_init[cam_ind]  = p_wr.copy()

                    self.tf_wm_init_msg[cam_ind].transform.rotation = Quaternion(*self.q_wm_init[cam_ind] )
                    self.tf_wm_init_msg[cam_ind].transform.translation.x = self.p_wm_init[cam_ind][0]
                    self.tf_wm_init_msg[cam_ind].transform.translation.y = self.p_wm_init[cam_ind][1]
                    self.tf_wm_init_msg[cam_ind].transform.translation.z = self.p_wm_init[cam_ind][2]    

                    self.q_wm_init_inv[cam_ind]  = inv_(self.q_wm_init[cam_ind]) 
                    self.p_wm_init_inv[cam_ind]  = -rot_by_q(self.p_wm_init[cam_ind], 
                                                             self.q_wm_init_inv[cam_ind]) 
                    self.q_or[cam_ind] = ident_q
                    self.p_or[cam_ind] = np.zeros(4)

                    rospy.loginfo(f"Found init odom tf for cam{cam_ind}")
                else:
                    q_or = q_mult(self.q_wm_init_inv[cam_ind], q_wr)
                    p_or = rot_by_q(p_wr, self.q_wm_init_inv[cam_ind]) + self.p_wm_init_inv[cam_ind] 

                    self.q_or_dt[cam_ind] = q_mult(inv_(self.q_or[cam_ind]), q_or)
                    self.p_or_dt[cam_ind] = rot_by_q(p_or, inv_(self.q_or[cam_ind])) - rot_by_q(self.p_or[cam_ind], inv_(self.q_or[cam_ind])) 

                    self.q_or[cam_ind] = q_or
                    self.p_or[cam_ind] = p_or

                self.tf_or_msg[cam_ind].transform.rotation = Quaternion(*self.q_or[cam_ind]) 
                self.tf_or_msg[cam_ind].transform.translation.x = self.p_or[cam_ind][0]
                self.tf_or_msg[cam_ind].transform.translation.y = self.p_or[cam_ind][1]
                self.tf_or_msg[cam_ind].transform.translation.z = self.p_or[cam_ind][2]

                # print("quat:",[f"{q0-q1:.4f}" for q0, q1 in zip(self.q_or_dt[0],self.q_or_dt[1])])
                # print("tran:",[f"{p0-p1:.4f}" for p0, p1 in zip(self.p_or_dt[0],self.p_or_dt[1])])
                # print(f"cam{cam_ind}: {np.linalg.norm(self.p_or_dt[cam_ind],1):.4f}")

                if self.p_dt_norms_ptr<1500 and self.p_dt_norms_ptr>-1:
                    if cam_ind==0: 
                        # if self.tf_time_delta[cam_ind]<0.1:
                        self.p_dt_norms[self.p_dt_norms_ptr] = np.linalg.norm(self.p_or_dt[cam_ind],1)
                        self.p_dt_time_deltas[self.p_dt_norms_ptr] = self.tf_time_delta[cam_ind]
                        self.p_dt_norms_ptr += 1
                else:
                    if self.p_dt_norms_ptr>0:
                        self.p_dt_norms_ptr = -1                  
                        print(f"p_dt_norms: max={np.max(self.p_dt_norms):.4f}, min={np.min(self.p_dt_norms):.4f}, mean={np.mean(self.p_dt_norms):.4f} std={np.std(self.p_dt_norms):.4f}, median={np.median(self.p_dt_norms):.4f}")



    def fid_tf_cb(self, fid_msg: FiducialTransformArray):

        tfs_to_pub = []

        if self.tf_rl_msg.header.stamp.to_sec()-fid_msg.header.stamp.to_sec() >= 1:
            self.tf_rl_msg.header.stamp = fid_msg.header.stamp  
            tfs_to_pub.append(self.tf_rl_msg)

        cam_ind = int(fid_msg.header.frame_id[-1])
                
        self.tf_time_delta[cam_ind] = fid_msg.header.stamp.to_sec()-self.tf_time_delta[cam_ind] 

        self.tf_mo_init_msg.header.stamp = fid_msg.header.stamp

        self.tf_wm_init_msg[cam_ind].header.stamp = fid_msg.header.stamp
        self.tf_or_msg[cam_ind].header.stamp = fid_msg.header.stamp
        self.tf_wc_msg[cam_ind].header.stamp = fid_msg.header.stamp        

        if len(fid_msg.transforms)>0:

            dt_start = (rospy.Time.now().to_sec()-self.start_time)
            dt_map = (rospy.Time.now().to_sec()-self.map_got_time)

            self.process_tf_msg(fid_msg, cam_ind)

            if self.world_found[cam_ind]:
                tfs_to_pub += [
                                self.tf_or_msg[cam_ind],
                                self.tf_wm_init_msg[cam_ind], 
                                self.tf_wc_msg[cam_ind], 
                                ]
                
                if cam_ind==0:
                    maps_too_old = True #dt_map>self.no_map_timeout
                    starting = dt_start<self.no_map_timeout
                    odom_pub_cond = starting or maps_too_old

                    if odom_pub_cond:
                        tfs_to_pub.append(self.tf_mo_init_msg)
                        
                        if self.odom_pub_stopped:
                            self.odom_pub_stopped = False
                            rospy.loginfo("Started sending map->odom frame")
                    else:
                        if not self.odom_pub_stopped:
                            self.odom_pub_stopped = True
                            rospy.loginfo("Map is building! Stopped sending map->odom frame")

                self.tf_broadcaster0.sendTransform(tfs_to_pub)



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

    # plt.hist(robot_tf_publisher.p_dt_time_deltas, bins="auto")
    plt.hist(robot_tf_publisher.p_dt_norms, bins="auto")    
    plt.show()