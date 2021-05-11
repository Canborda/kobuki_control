#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

class reference_listener:

    def __init__(self):

        # Publisher
        self.pub_pose = rospy.Publisher("/add_reference_topic", Pose2D, queue_size=20)
        self.info = rospy.Publisher("/info_topic", String, queue_size=20)

        rospy.loginfo('>> REFERENCE LISTENER SAYS: Enter reference points (x,y)')
        self.info.publish('------------------------------------------------')
        self.info.publish('>> REFERENCE LISTENER SAYS: new session started.')

        while True:
            ref = input('>> ')
            
            # Kill node
            if ref == 'q':
                rospy.loginfo('>> REFERENCE LISTENER SAYS: node killed.')
                rospy.signal_shutdown('Request shutdown')
                break
            # Assignment routine
            elif ref == 'saved_1':
                self.saved_routine_1()
            # Send points
            else:
                self.string2reflist(ref)

    def string2reflist(self, ref):
        ref = ref.split(' ')
        for point_str in ref:
            # Parse strings
            point_list = point_str.replace('(',' ').replace(')',' ').replace(',',' ').split(' ')
            # Create Pose2D message
            pose_ref = Pose2D()
            pose_ref.x = float(point_list[1])
            pose_ref.y = float(point_list[2])
            self.pub_pose.publish(pose_ref)

    def saved_routine_1(self):
        ref =  '(-1.0,0.0) (-3.5,0.0) (-3.5,3.5) (1.5,3.5) (1.5,-1.5) (3.5,-1.5) '
        ref += '(3.5,-8.0) (-2.5,-8.0) (-2.5,-5.5) (1.5,-5.5) (1.5,-3.5) (-1.0,-3.5)'
        self.string2reflist(ref)


if __name__ == '__main__':
    rospy.init_node("reference_listener_node", anonymous=True)
    rospy.sleep(2)
    rospy.loginfo('>> REFERENCE LISTENER SAYS: node initialized.')
    reference_listener()
    rospy.spin()