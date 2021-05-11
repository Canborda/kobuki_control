#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String

from collections import deque

class reference_manager:

    def __init__(self):

        self.stopped = False

        # Subscribers
        self.sub_new_points = rospy.Subscriber("/add_reference_topic", Pose2D, self.add_point, queue_size=10)
        self.sub_flag = rospy.Subscriber("/next_pose_flag_topic", Bool, self.next_point, queue_size=10)

        # Publisher
        self.pub_next_pose = rospy.Publisher("/current_reference_topic", Pose2D, queue_size=10)
        self.info = rospy.Publisher("/info_topic", String, queue_size=20)

        # Create references queue
        self.pose_queue = deque()

    def add_point(self, pose):
        self.pose_queue.append(pose)
        self.info.publish('>> REFERENCE MANAGER SAYS: point ({},{}) added. Queued points = {}.'
        .format(pose.x, pose.y, len(self.pose_queue)))
    
    def next_point(self, flag):
        if len(self.pose_queue) > 0:
            self.stopped = True
            msg = self.pose_queue.popleft()
            self.pub_next_pose.publish(msg)
            self.info.publish('>> REFERENCE MANAGER SAYS: reference changed! New reference = ({},{}).'
            .format(msg.x, msg.y))
        elif self.stopped:
            self.stopped = False
            self.info.publish('>> REFERENCE MANAGER SAYS: All references reached! Robot waiting...')


if __name__ == '__main__':
    rospy.init_node("reference_manager_node", anonymous=True)
    rospy.loginfo('>> REFRENCE MANAGER SAYS: node initialized.')
    reference_manager()
    rospy.spin()