import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose2D

from collections import deque

pose_queue = deque()

ref =  '(-1.0,0.0) (-3.5,0.0) (-3.5,3.5) (1.5,3.5) (1.5,-1.5) (3.5,-1.5) '
ref += '(3.5,-8.0) (-2.5,-8.0) (-2.5,-5.5) (1.5,-5.5) (1.5,-3.5) (-1.0,-3.5)'
print(ref)
ref = ref.split(' ')
for point_str in ref:
    # Parse strings
    point_list = point_str.replace('(',' ').replace(')',' ').replace(',',' ').split(' ')
    # Create Pose2D message
    pose_ref = Pose2D()
    pose_ref.x = float(point_list[1])
    pose_ref.y = float(point_list[2])
    pose_queue.append(pose_ref)
    print(pose_ref)

print('---')
print(str(pose_queue))