#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
from numpy import sign, int8, sqrt


def clip(val: float, max_val: float):
    return val if abs(val)<max_val else max_val*sign(val)



'''
0 ------ 1
----------
----------
----------
3 ------ 2
'''

class joy_processor:
    def __init__(self):
        rospy.Subscriber("joy", Joy, self.callback)

        self.pub = rospy.Publisher("/sisyph/wheel_cmd", Int8MultiArray, queue_size=100)

        self.chassis_L = 0.168
        self.chassis_W = 0.25
        self.chassis_dim_sum = self.chassis_L + self.chassis_W
        self.wheel_R = 0.1
        self.max_vx = -1
        self.max_vy = 1 
        self.min_vx = -0.25
        self.min_vy = 0.25
        self.max_wz = 1.3
        self.max_wheel_U = (self.chassis_dim_sum * self.max_wz + sqrt(2)*self.max_vx/2 + sqrt(2)*self.max_vy/2)/self.wheel_R 
        self.max_wheel_cmd = 60
        print(f"self.max_wheel_U: {self.max_wheel_U}")

        self.w_z = 0.0
        self.v_x = 0.0
        self.v_y = 0.0





    def callback(self, data):
        # print(f"data.axes[0]: {data.axes[0]}, data.axes[1]: {data.axes[1]}, data.axes[2]: {data.axes[2]}, data.axes[3]: {data.axes[3]}, , data.axes[4]: {data.axes[4]}, , data.axes[5]: {data.axes[5]}")

        self.w_z = self.max_wz*data.axes[3]
        self.v_x = self.max_vx*data.axes[1]
        self.v_y = self.max_vy*data.axes[0] 

        if data.axes[1]==0 and data.axes[0]==0:
            self.v_x = self.min_vx*data.axes[7]
            self.v_y = self.min_vy*data.axes[6] 

        # print((self.chassis_dim_sum * self.w_z + self.v_x + self.v_y)/self.wheel_R)

        wheel_cmd_0 = (-self.chassis_dim_sum*self.w_z + self.v_x - self.v_y)/self.wheel_R
        wheel_cmd_1 = (self.chassis_dim_sum*self.w_z + self.v_x + self.v_y)/self.wheel_R
        wheel_cmd_2 = (self.chassis_dim_sum*self.w_z + self.v_x - self.v_y)/self.wheel_R
        wheel_cmd_3 = (-self.chassis_dim_sum*self.w_z + self.v_x + self.v_y)/self.wheel_R

        # print(f"wheel_cmd_0: {wheel_cmd_0}, wheel_cmd_1: {wheel_cmd_1}, wheel_cmd_2: {wheel_cmd_2}, wheel_cmd_3: {wheel_cmd_3}")

        wheel_cmd_0 = (clip(wheel_cmd_0/self.max_wheel_U, 1))*self.max_wheel_cmd
        wheel_cmd_1 = (clip(wheel_cmd_1/self.max_wheel_U, 1))*self.max_wheel_cmd
        wheel_cmd_2 = (clip(wheel_cmd_2/self.max_wheel_U, 1))*self.max_wheel_cmd
        wheel_cmd_3 = (clip(wheel_cmd_3/self.max_wheel_U, 1))*self.max_wheel_cmd

        # print(f"scaled wheel_cmd_0: {wheel_cmd_0}, scaled wheel_cmd_1: {wheel_cmd_1}, scaled wheel_cmd_2: {wheel_cmd_2}, scaled wheel_cmd_3: {wheel_cmd_3}")

        # print(f"int8 wheel_cmd_0: {int8(wheel_cmd_0)}, int8 wheel_cmd_1: {int8(wheel_cmd_1)}, int8 wheel_cmd_2: {int8(wheel_cmd_2)}, int8 wheel_cmd_3: {int8(wheel_cmd_3)}")

        bot_cmd = Int8MultiArray()
        bot_cmd.data = [0,0,0,0,0]
        bot_cmd.data[0] = int8(wheel_cmd_0)
        bot_cmd.data[1] = int8(wheel_cmd_1)
        bot_cmd.data[2] = int8(wheel_cmd_2)
        bot_cmd.data[3] = int8(wheel_cmd_3)

        bot_cmd.data[4] = 0
        if data.buttons[8]:
            bot_cmd.data[4] = 1

        self.pub.publish(bot_cmd)


  



if __name__ == '__main__':
    rospy.init_node('TestControlOneComputer17_06', anonymous=True)
    joy_proc = joy_processor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

        