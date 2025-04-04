#!/usr/bin/env python3
import rospy
from robot_multi_tf_pub import SisyphStatePublisher


if __name__ == '__main__':

    node_handler = rospy.init_node("sisyph_state_pub", anonymous=False)
    robot_tf_publisher = SisyphStatePublisher(node_handler, 46, 42, [0.2555, 0.078])

    # static_tf_broadc = tf2_ros.TransformBroadcaster()

    try:
        rospy.spin()
        # while not rospy.core.is_shutdown():
        #     robot_tf_publisher.tf_rl_msg.header.stamp = rospy.Time.now()
        #     static_tf_broadc.sendTransform(robot_tf_publisher.tf_rl_msg)
        #     rospy.sleep(0.04)
    except rospy.ROSInterruptException:
        pass

