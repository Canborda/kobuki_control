#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from collections import deque

class reference_manager:

    def __init__(self):

        # Subscribers
        self.sub_new_points = rospy.Subscriber("/add_reference_topic", Pose2D, self.add_point, queue_size=10)
        self.sub_flag = rospy.Subscriber("/next_pose_flag_topic", Bool, self.next_point, queue_size=10)

        # Publisher
        self.pub_next_pose = rospy.Publisher("/current_reference_topic", Pose2D, queue_size=10)

        self.pose_queue = deque()

    def add_point(self, pose):
        self.pose_queue.append(pose)
        rospy.loginfo('>> REFERENCE MANAGER SAYS: Point added. New queued points = {}'.format(len(self.pose_queue)))
    
    def next_point(self, flag):
        if len(self.pose_queue) > 0:
            msg = self.pose_queue.popleft()
            self.pub_next_pose.publish(msg)
            rospy.loginfo('>> REFERENCE MANAGER SAYS: Reference changed! New reference is:')
            print(msg)
        else:
            rospy.loginfo('>> REFERENCE MANAGER SAYS: All references reached! Robot waiting...')


if __name__ == '__main__':
    rospy.init_node("reference_manager_node", anonymous=True)
    rospy.loginfo('>> STATUS: Initialize \"reference_manager\" node')
    reference_manager()
    rospy.spin()