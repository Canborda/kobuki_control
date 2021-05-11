#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np

class velocity_publisher:

    def __init__(self):
        
        # Robot model parameters

        r = 0.035
        l = 0.23/2

        left_alpha = np.pi/2
        left_betha = 0.0

        right_alpha = -np.pi/2
        right_betha = np.pi

        # Jacobian
        
        J1 = np.array([[np.sin(left_alpha+left_betha), -np.cos(left_alpha+left_betha), -l*np.cos(left_betha)],
                       [np.sin(right_alpha+right_betha), -np.cos(right_alpha+right_betha), -l*np.cos(right_betha)]])

        J2 = r*np.identity(2)

        self.Jacobian = np.matmul(np.linalg.pinv(J2), J1)

        # Subscriber
        self.sub_cmdvel = rospy.Subscriber("/velocity_topic", Twist, self.cmd_velocity, queue_size=10)

        # Publisher
        self.pub_left_wheel = rospy.Publisher("/wheel_left_ctrl/command", Float64, queue_size=10)
        self.pub_right_wheel = rospy.Publisher("/wheel_right_ctrl/command", Float64, queue_size=10)

    def cmd_velocity(self, cmd_vel):

        # Inverse kinematics
        result = np.matmul(self.Jacobian, [cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z*np.pi/180])
        
        # Send velocity

        msgFloat = Float64()

        msgFloat.data = result[0]
        self.pub_left_wheel.publish(msgFloat)

        msgFloat.data = result[1]
        self.pub_right_wheel.publish(msgFloat)

if __name__ == '__main__':
    rospy.init_node("model_node", anonymous=True)
    rospy.loginfo('>> MODEL NODE SAYS: node initialized.')
    velocity_publisher()
    rospy.spin()