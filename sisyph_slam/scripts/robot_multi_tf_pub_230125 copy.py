#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from visualization_msgs.msg import Marker
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


def tf_to_pose(pose, transform):
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w    
    return pose



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
        self.tf_mo_msg = [TransformStamped(), TransformStamped()]
        for i_cam_, cam_ in enumerate(self.cams):
            self.tf_wc_msg[i_cam_].header.frame_id = "world" 
            self.tf_wc_msg[i_cam_].child_frame_id = cam_
            self.tf_wc_msg[i_cam_].transform.rotation = Quaternion(*ident_q)

            self.tf_mo_msg[i_cam_].header.frame_id = "map" 
            self.tf_mo_msg[i_cam_].child_frame_id = "odom"  
            self.tf_mo_msg[i_cam_].transform.rotation = Quaternion(*ident_q)

        self.tf_or_msg = TransformStamped() 
        self.tf_or_msg.header.frame_id = "odom" 
        self.tf_or_msg.child_frame_id = "robot" 
        self.tf_or_msg.transform.rotation = Quaternion(*ident_q)

        self.tf_wm_msg = TransformStamped() 
        self.tf_wm_msg.header.frame_id = "world" 
        self.tf_wm_msg.child_frame_id = "map"
        self.tf_wm_msg.transform.rotation = Quaternion(*ident_q)

        self.tf_rl_msg = TransformStamped()
        self.tf_rl_msg.header.frame_id = "robot" 
        self.tf_rl_msg.child_frame_id = "laser"  
        self.tf_rl_msg.transform.rotation = Quaternion(*ident_q)
        self.tf_rl_msg.transform.translation.x = 0.2555
        self.tf_rl_msg.transform.translation.y = 0.078
        
        self.pos_track = Marker()
        self.pos_track.header.frame_id = "world"
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
        self.pos_track_pub = rospy.Publisher("/pos_track", Marker)
        


        self.robot_found = np.array([False,False])
        self.world_found = np.array([False,False])

        self.q_cw_inv_init = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_cw_inv_init = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.q_mo = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_mo = [np.array([0,0,0,0]),np.array([0,0,0,0])]

        self.q_mo_d = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_mo_d = [np.array([0,0,0,0]),np.array([0,0,0,0])]        
        self.q_mo_dd = [np.array([0,0,0,1]),np.array([0,0,0,1])]
        self.p_mo_dd = [np.array([0,0,0,0]),np.array([0,0,0,0])]     

        self.tf_last_time = [0.0, 0.0]

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
                    rospy.loginfo(f"Found world for cam{cam_ind}")
                    self.world_found[cam_ind] = True

                    self.q_cw_inv_init[cam_ind] = inv_(tf_q) 
                    self.p_cw_inv_init[cam_ind] = -rot_by_q(tf_p, self.q_cw_inv_init[cam_ind]) 

                    self.tf_wc_msg[cam_ind].transform.rotation = Quaternion(*self.q_cw_inv_init[cam_ind])
                    self.tf_wc_msg[cam_ind].transform.translation.x = self.p_cw_inv_init[cam_ind][0]
                    self.tf_wc_msg[cam_ind].transform.translation.y = self.p_cw_inv_init[cam_ind][1]
                    self.tf_wc_msg[cam_ind].transform.translation.z = self.p_cw_inv_init[cam_ind][2]

            if fid_tf.fiducial_id == self.fid_robot and self.world_found[cam_ind]:   # world -> obot, world->map, map->odom, 
                if not self.robot_found[cam_ind]:
                    self.robot_found[cam_ind] = True
                    rospy.loginfo(f"Found robot for cam{cam_ind}")                

                q_mo = q_mult(self.q_cw_inv_init[cam_ind], tf_q)
                q_mo = q_mult(q_mo, flipz_q)
                q_mo = q_axis(q_to_euler(q_mo)[2],[0,0,1]) 

                p_mo = rot_by_q(tf_p, self.q_cw_inv_init[cam_ind]) + self.p_cw_inv_init[cam_ind]
                p_mo[2] = 0.0 

                q_mo_d = q_mult(inv_(self.q_mo[cam_ind]), q_mo)
                p_mo_d = rot_by_q(p_mo, inv_(self.q_mo[cam_ind])) - rot_by_q(self.p_mo[cam_ind], inv_(self.q_mo[cam_ind]))  
                self.q_mo_dd[cam_ind] = q_mult(inv_(self.q_mo_d[cam_ind]), q_mo_d)
                self.p_mo_dd[cam_ind] = rot_by_q(p_mo_d, inv_(self.q_mo_d[cam_ind])) - rot_by_q(self.p_mo_d[cam_ind], inv_(self.q_mo_d[cam_ind]))                

                self.q_mo_d[cam_ind] = q_mo_d
                self.p_mo_d[cam_ind] = p_mo_d
                self.q_mo[cam_ind] = q_mo
                self.p_mo[cam_ind] = p_mo

                self.tf_mo_msg[cam_ind].transform.rotation = Quaternion(*self.q_mo[cam_ind])
                self.tf_mo_msg[cam_ind].transform.translation.x = self.p_mo[cam_ind][0]
                self.tf_mo_msg[cam_ind].transform.translation.y = self.p_mo[cam_ind][1]
                self.tf_mo_msg[cam_ind].transform.translation.z = self.p_mo[cam_ind][2]    

                # print("quat:",[f"{q0-q1:.4f}" for q0, q1 in zip(self.q_mo_dt[0],self.q_mo_dt[1])])
                # print("p0-p1:",[float(f"{p0-p1:.3f}") for p0, p1 in zip(self.p_mo_d[0],self.p_mo_d[1])])
                # print(f"cam0: {np.linalg.norm(self.p_mo_dt[0],1):.4f}, cam1: {np.linalg.norm(self.p_mo_dt[1],1):.4f}")
                # print("p01 dev:",[float(f"{p_:.3f}") for p_ in p_mo01_dev])
                # print("cam0 q:",[f"{q_:.4f}" for q_ in self.q_mo_d[0]])

                if self.world_found.all():                                       
                    if cam_ind==1:

                        dt0_ = tf_time-self.tf_last_time[cam_ind]

                        q0_d= abs(q_to_euler(self.q_mo_d[cam_ind])[2])
                        q0_dd = abs(q_to_euler(self.q_mo_dd[cam_ind])[2])
                        q0_ddt_ = q0_dd*dt0_>0.005

                        p0_ddt_ = dt0_*np.max(np.abs(self.p_mo_dd[cam_ind]))
                        p0_dt_ = dt0_*np.max(np.abs(self.p_mo_d[cam_ind]))
                        p0_vel = np.max(np.abs(self.p_mo_d[cam_ind])) + p0_ddt_
                        p0dt_cond = p0_ddt_>0.002

                        new_pub_tfs_from = 1
                        if p0dt_cond or q0_ddt_:
                            new_pub_tfs_from = -1

                        # formatter_="{:.03f}".format
                        # with np.printoptions(precision=3, formatter={"float":formatter_}):

                        # print(f"p0_vel: {p0_vel:.4f}")
                        print(f"q0_dd*dt0: {q0_dd*dt0_:.4f}")
                        # print(f"p0_dt {p0_dt_:.03f}, p0_ddt {p0_ddt_:.03f}, p0_ddt*p0_dt {p0_ddt_*p0_dt_:.05f}")
                        # if new_pub_tfs_from!=self.pub_tfs_from:
                            # print(f"{self.pub_tfs_from}-->{new_pub_tfs_from}, dt0: {tf_time-self.tf_last_time[0]:.03f}, dt0*p0_dd {(tf_time-self.tf_last_time[0])*np.abs(np.max(self.p_mo_dd[0])):.03f}")

                        self.pub_tfs_from = new_pub_tfs_from

                else: 
                    self.pub_tfs_from = 1





    def fid_tf_cb(self, fid_msg: FiducialTransformArray):

        tf_time = fid_msg.header.stamp.to_sec()

        tfs_to_pub = []

        cam_ind = int(fid_msg.header.frame_id[-1])
        # if self.world_found.all() and cam_ind==0 and self.robot_found.all(): return

        if len(fid_msg.transforms)>0:

            self.process_tf_msg(fid_msg.transforms, cam_ind, tf_time)

            if self.world_found[cam_ind]:

                self.tf_wc_msg[cam_ind].header.stamp = fid_msg.header.stamp        
                tfs_to_pub.append(self.tf_wc_msg[cam_ind])                
                
                if not self.pub_tfs_from==-1:
                    if cam_ind==self.pub_tfs_from:

                        self.tf_or_msg.header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_or_msg)

                        self.tf_wm_msg.header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_wm_msg)

                        self.tf_rl_msg.header.stamp = fid_msg.header.stamp  
                        tfs_to_pub.append(self.tf_rl_msg)

                        self.tf_mo_msg[cam_ind].header.stamp = fid_msg.header.stamp
                        tfs_to_pub.append(self.tf_mo_msg[cam_ind])

                        pos_param = self.pos_track.id/self.max_pos_track  
                        color_ = np.abs(np.array([np.cos(pos_param*np.pi), np.sin(pos_param*np.pi*2), pos_param]))
                        color_ /= np.linalg.norm(color_)
                        self.pos_track.color.r = color_[0]
                        self.pos_track.color.g = color_[1]
                        self.pos_track.color.b = color_[2]
                        self.pos_track.pose = tf_to_pose(self.pos_track.pose, self.tf_mo_msg[cam_ind].transform)
                        self.pos_track.id += 1
                        if self.pos_track.id > self.max_pos_track: 
                            self.pos_track.action = Marker.DELETEALL
                            self.pos_track.id = -1
                        self.pos_track_pub.publish(self.pos_track)
                        self.pos_track.action = Marker.ADD

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

