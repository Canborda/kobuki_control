#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D

class reference_listener:

    def __init__(self):

        # Publisher
        self.pub_pose = rospy.Publisher("/add_reference_topic", Pose2D, queue_size=10)

        while True:

            ref = input('Enter reference points (q for quit): ')
            ref = ref.split(' ')
            print(ref)

            # Kill node
            if ref[0] == 'q':
                rospy.loginfo('>> STATUS: Quit.')
                rospy.signal_shutdown('Request shutdown')
                break

            else:
                for point_str in ref:
                    # Parse strings
                    point_list = point_str.replace('(',' ').replace(')',' ').replace(',',' ').split(' ')
                    # Create Pose2D message
                    pose_ref = Pose2D()
                    pose_ref.x = float(point_list[1])
                    pose_ref.y = float(point_list[2])
                    pose_ref.theta = 0.0    #TODO ask theta?
                    self.pub_pose.publish(pose_ref)


if __name__ == '__main__':
    rospy.init_node("reference_listener_node", anonymous=True)
    rospy.loginfo('>> STATUS: Initialize \"reference_listener\" node')
    reference_listener()
    rospy.spin()