#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

import numpy as np

class pose_subscriber:

    def __init__(self):
        
        # Subscriber
        self.sub_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.get_pose, queue_size=10)

        # Publisher
        self.pub_pose = rospy.Publisher("/current_pose_topic", Pose2D, queue_size=10)


    def get_pose(self, data):
        
        pose_msg = Pose2D()

        # Get position
        pose_msg.x = data.pose[2].position.x
        pose_msg.y = data.pose[2].position.y

        # Get rotation
        quat = data.pose[2].orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        pose_msg.theta = yaw*180/np.pi

        # Publish (x,y,ϴ)
        self.pub_pose.publish(pose_msg)

        print('\n','-'*50,'\n')
        print(pose_msg)

if __name__ == '__main__':
    rospy.init_node("pose_node", anonymous=True)
    rospy.loginfo('>> STATUS: Initialize \"pose\" node')
    pose_subscriber()
    rospy.spin()