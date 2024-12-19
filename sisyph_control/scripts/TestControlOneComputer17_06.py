#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Char

def callback(data):
    
    commands = {
        'F': data.axes[1] > 0.5,  # Forward
        'B': data.axes[1] < -0.5, # Backward
        'L': data.axes[0] < -0.5, # Left
        'R': data.axes[0] > 0.5,  # Right
        'I': data.buttons[5],     # Forward-right
        'G': data.buttons[4],     # Forward-left
        'J': data.buttons[7],     # Backward-right
        'H': data.buttons[6],     # Backward-left
        'X': data.buttons[0],     # Rotate clockwise
        'Y': data.buttons[1],     # Rotate counter-clockwise
        'S': data.buttons[8]      # Stop
    }

    for command, condition in commands.items():
        if condition:
            pub.publish(ord(command))
            break

def TestControlOneComputer17_06():
    rospy.init_node('TestControlOneComputer17_06', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    global pub
    pub = rospy.Publisher('sisyph_control', Char, queue_size=100)
    rospy.spin()

if __name__ == '__main__':
    TestControlOneComputer17_06()
