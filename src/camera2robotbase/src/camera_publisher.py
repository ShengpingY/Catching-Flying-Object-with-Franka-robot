#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray   
import numpy as np
import random

class CameraPublisherNode:

    def __init__(self):
        rospy.init_node('camera_publisher')
        self.publisher_position_cam_frame = rospy.Publisher('/pos_in_cam_frame', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.i = 0

    def get_coordinates_of_object(self):
        x = random.randint(250, 450) 
        y = random.randint(250, 450)
        z = random.randint(250, 450) 

        object_position = np.array([x, y, z])

        self.publish_coordinates(object_position)

        self.i += 1

    def publish_coordinates(self, position):
        msg = Float64MultiArray()
        msg.data = position.tolist()
        self.publisher_position_cam_frame.publish(msg)

def main():
    camera_publisher = CameraPublisherNode()

    while not rospy.is_shutdown():
        camera_publisher.get_coordinates_of_object()
        camera_publisher.rate.sleep()

if __name__ == '__main__':
    main()
