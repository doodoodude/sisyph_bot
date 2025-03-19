#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_conjugate as q_conj
from tf.transformations import quaternion_from_euler as euler_to_q
from tf.transformations import euler_from_quaternion as q_to_euler
from tf.transformations import quaternion_about_axis as q_axis
from tf.transformations import quaternion_inverse as inv_
from tf.transformations import quaternion_multiply as q_mult


def rot_by_q(_vect, _q):
    return q_mult(q_mult(_q, _vect), q_conj(_q))

def tf_to_pose(pose, transform):
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.orientation.x = transform.rotation.x
    pose.orientation.y = transform.rotation.y
    pose.orientation.z = transform.rotation.z
    pose.orientation.w = transform.rotation.w    
    return pose


def qp_from_tf_msg(_msg):
    return np.array([_msg.transform.rotation.x, _msg.transform.rotation.y, _msg.transform.rotation.z, _msg.transform.rotation.w]), np.array([_msg.transform.translation.x, _msg.transform.translation.y, _msg.transform.translation.z, 0.0])

def qp_to_tf_msg(qp_:tuple, _msg):
    _msg.transform.rotation = Quaternion(*qp_[0])
    _msg.transform.translation.x = qp_[1][0]
    _msg.transform.translation.y = qp_[1][1]
    _msg.transform.translation.z = qp_[1][2]       
    return _msg  

def qp_inv(qp_:tuple):
    inv_q = inv_(qp_[0])
    return (inv_q,-rot_by_q(qp_[1], inv_q))

def qp_mult(qp_, other_qp_):
    return (q_mult(qp_[0], other_qp_[0]), rot_by_q(other_qp_[1], qp_[0]) + qp_[1])    

def qp_relative(qp_init_, qp_final_):
    return qp_mult(qp_inv(qp_init_), qp_final_)


    # q_or = q_mult(self.q_wm_init_inv[cam_ind], self.q_wr[cam_ind])
    # p_or = rot_by_q(self.p_wr[cam_ind], self.q_wm_init_inv[cam_ind]) + self.p_wm_init_inv[cam_ind] 



correct_fid_q = euler_to_q(-3.1414,-3.1414,0)
ident_q = np.array([0, 0, 0, 1])
flipz_q = q_axis(3.1414, (0,0,1))


