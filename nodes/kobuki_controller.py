#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool, String, Float32MultiArray

import numpy as np

class controller:

    def __init__(self):

        # PARAMS
        self.MIN_ERROR = 0.1        # Minimum accepted error to change reference
        self.MIN_STOP = 0.01        # Minimum accepted error to stop robot
        self.MIN_ERROR_FLAG = 0.2   # Minimum error to restart asking references
        self.V_C = 0.3              # Linear cruise velocity (m/s)
        self.W_C = 60               # Angular cruise velocity (deg/s)

        self.error_flag = True      # Flag to restart asking references
        self.reference = Pose2D()

        # Initial reference
        initial_pose = Pose2D()
        initial_pose.x = -1.0
        initial_pose.y = 0.0
        self.reference = initial_pose

        # Subscribers
        self.sub_ref = rospy.Subscriber("/current_reference_topic", Pose2D, self.change_reference, queue_size=10)
        self.sub_pose = rospy.Subscriber("/current_pose_topic", Pose2D, self.get_error, queue_size=10)

        # Publisher
        self.pub_flag = rospy.Publisher("/next_pose_flag_topic", Bool, queue_size=10)
        self.pub_velocity = rospy.Publisher("/velocity_topic", Twist, queue_size=10)
        self.pub_error = rospy.Publisher("/error_topic", Float32MultiArray, queue_size=10)
        
        self.info = rospy.Publisher("/info_topic", String, queue_size=20)


    def change_reference(self, new_ref):
        self.reference = new_ref

    def get_error(self, pose):

        # Calculate errors
        error = Pose2D()
        error.x = pose.x - self.reference.x
        error.y = pose.y - self.reference.y

        self.reference.theta = np.arctan2(-error.y,-error.x)*180/np.pi
        error.theta = self.reference.theta - pose.theta
        if error.theta > 180: error.theta -= 360
        if error.theta < -180: error.theta += 360

        # Linear controller
        error_pos = np.sqrt(error.x**2 + error.y**2)
        Vx = 2*self.V_C / ( 1 + np.exp(-20*error_pos) ) - self.V_C

        # Angular controller
        Wz = 2*self.W_C / ( 1 + np.exp(-error.theta/5) ) - self.W_C
        
        # Wait flag for ask new reference
        if error_pos > self.MIN_ERROR_FLAG:     # Must be MIN_ERROR_FLAG > MIN_ERROR
            self.error_flag = True

        # Ask for new reference
        if error_pos < self.MIN_ERROR:
            if self.error_flag:
                self.pub_flag.publish(True)
                self.error_flag = False

        # Stop
        if error_pos < self.MIN_STOP:
            self.error_flag = True
            Vx = 0.0
            Wz = 0.0

        # Publish velocity
        msg = Twist()
        msg.linear.x = Vx
        msg.angular.z = Wz
        self.pub_velocity.publish(msg)

        # Publish error
        error_msg = Float32MultiArray()
        error_msg.data = [error.x, error.y, error_pos, error.theta]
        self.pub_error.publish(error_msg)

if __name__ == '__main__':
    rospy.init_node("kobuki_controller_node", anonymous=True)
    rospy.loginfo('>> KOBUKI CONTROLLER SAYS: node initialized.')
    controller()
    rospy.spin()