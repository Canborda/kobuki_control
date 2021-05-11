#!/usr/bin/python3

import rospy
from std_msgs.msg import String

class info_subscriber:

    def __init__(self):
        
        # Subscriber
        self.sub_pose = rospy.Subscriber("/info_topic", String, self.show_info, queue_size=10)

        rospy.loginfo('>> Listening...')

    def show_info(self, data):
        print(data.data)

if __name__ == '__main__':
    rospy.init_node("info_node", anonymous=True)
    rospy.sleep(2)
    rospy.loginfo('>> INFO SUBSCRIBER SAYS: node initialized.')
    info_subscriber()
    rospy.spin()