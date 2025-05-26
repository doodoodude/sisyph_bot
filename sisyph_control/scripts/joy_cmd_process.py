#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
import numpy as np

np.set_printoptions(precision=4)

def clip(val: float, max_val: float):
    return val if abs(val)<max_val else max_val*np.sign(val)


def clip_norm(vec: np.ndarray, eps:float=1e-8):
    norm = np.linalg.norm(vec,2)
    return vec*(np.clip(norm,0,1)/(norm+eps))


# robot schematic
''' W x 2
0 ------ 1
----------
---------- L x 2
----------
3 ------ 2 R
'''




if __name__ == '__main__':
    rospy.init_node("joy_cmd_process", anonymous=False)

    pub = rospy.Publisher("/sisyph/wheel_cmd", Int8MultiArray, queue_size=100)

    joy_deadzone = 0.12

    chassis_L = 0.15895
    chassis_W = 0.96
    chassis_dim_sum = chassis_L + chassis_W
    wheel_R = 0.1
    min_vx,min_vy = 0.01, 0.01
    joy_pow_x, joy_pow_y, joy_pow_w = 2,2,2
    fast_mode = False

    max_vx, max_vy, max_wz = 1,1,1
    max_wheel_U = (chassis_dim_sum * abs(max_wz) + np.sqrt(2)*abs(max_vx)/2 + np.sqrt(2)*abs(max_vy)/2)/wheel_R 
    max_wheel_cmd = 50
    max_fast_wheel_cmd = 100

    print(f"max_wheel_U: {max_wheel_U}")


    def callback(data):
        global fast_mode

        # print(f"data.axes[0]: {data.axes[0]}, data.axes[1]: {data.axes[1]}, data.axes[2]: {data.axes[2]}, data.axes[3]: {data.axes[3]}, , data.axes[4]: {data.axes[4]}, , data.axes[5]: {data.axes[5]}")
 
        fast_mode = True if data.buttons[6] and not fast_mode else False if data.buttons[6] and fast_mode else fast_mode

        wheel_cmd_scale = max_fast_wheel_cmd if fast_mode else max_wheel_cmd

        v_x = abs(data.axes[1]**joy_pow_x)*np.sign(data.axes[1])
        v_y = abs(data.axes[0]**joy_pow_y)*np.sign(data.axes[0])
        w_z = abs(data.axes[3]**joy_pow_w)*np.sign(data.axes[3])

        v_x = min_vx*data.axes[7] if data.axes[1]==0 else v_x
        v_y = min_vy*data.axes[6] if  data.axes[0]==0 else v_y

        v_xy_cmd = clip_norm(np.array([v_x,v_y]))
        v_x, v_y = v_xy_cmd[0]*max_vx, v_xy_cmd[1]*max_vy
        w_z *= max_wz
        # print(v_xy_cmd, w_z)

        wheel_cmd_0 = (-chassis_dim_sum*w_z + v_x - v_y)/wheel_R
        wheel_cmd_1 = (chassis_dim_sum*w_z + v_x + v_y)/wheel_R
        wheel_cmd_2 = (chassis_dim_sum*w_z + v_x - v_y)/wheel_R
        wheel_cmd_3 = (-chassis_dim_sum*w_z + v_x + v_y)/wheel_R

        # print(f"{wheel_cmd_0:.3f}, {wheel_cmd_1:.3f}, {wheel_cmd_2:.3f}, {wheel_cmd_3:.3f}")
        # print(f"{wheel_cmd_0/max_wheel_U:.3f}, {wheel_cmd_1/max_wheel_U:.3f}, {wheel_cmd_2/max_wheel_U:.3f}, {wheel_cmd_3/max_wheel_U:.3f}")

        wheel_cmd_0 = (np.clip(wheel_cmd_0/max_wheel_U, -1, 1))*wheel_cmd_scale
        wheel_cmd_1 = (np.clip(wheel_cmd_1/max_wheel_U, -1, 1))*wheel_cmd_scale
        wheel_cmd_2 = (np.clip(wheel_cmd_2/max_wheel_U, -1, 1))*wheel_cmd_scale
        wheel_cmd_3 = (np.clip(wheel_cmd_3/max_wheel_U, -1, 1))*wheel_cmd_scale

        # print(f"{wheel_cmd_0:.3f}, {wheel_cmd_1:.3f}, {wheel_cmd_2:.3f}, {wheel_cmd_3:.3f}")

        bot_cmd = Int8MultiArray()
        bot_cmd.data = [0,0,0,0,0]
        bot_cmd.data[0] = np.int8(wheel_cmd_3) # wheel 3
        bot_cmd.data[1] = np.int8(wheel_cmd_2) # wheel 2
        bot_cmd.data[2] = np.int8(wheel_cmd_1) # wheel 1 
        bot_cmd.data[3] = np.int8(wheel_cmd_0) # wheel 0

        bot_cmd.data[4] = 0
        if data.buttons[8]:
            bot_cmd.data[4] = 1

        pub.publish(bot_cmd)

    rospy.Subscriber("joy", Joy, callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

        
