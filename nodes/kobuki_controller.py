#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

import numpy as np

class controller:

    def __init__(self):

        self.MIN_ERROR = 0.1

        self.reference = Pose2D()

        # Subscribers
        self.sub_ref = rospy.Subscriber("/current_reference_topic", Pose2D, self.change_reference, queue_size=10)
        self.sub_pose = rospy.Subscriber("/current_pose_topic", Pose2D, self.get_error, queue_size=10)

        # Publisher
        self.pub_flag = rospy.Publisher("/next_pose_flag_topic", Bool, queue_size=10)
        #TODO cmd_vel publisher


    def change_reference(self, new_ref):
        self.reference = new_ref
        print('New ref:')
        print(self.reference)

    def get_error(self, pose):
        # error_x = self.reference.x - pose.x
        # error_y = self.reference.y - pose.y
        # error_rot = self.reference.theta - pose.theta

        # error_pos = np.sqrt(error_x**2 + error_y**2)

        # # Ask for new reference
        # if error_pos < self.MIN_ERROR:
        #     self.pub_flag.publish(True)
        e = input('Enter key:')
        if e == 'e':
            print('-'*50)
            print('Ref reached:')
            print(self.reference)
            self.pub_flag.publish(True)
            



if __name__ == '__main__':
    rospy.init_node("kobuki_controller_node", anonymous=True)
    rospy.loginfo('>> STATUS: Initialize \"kobuki_controller\" node')
    controller()
    rospy.spin()