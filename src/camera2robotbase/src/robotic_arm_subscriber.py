#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray 
import numpy as np
from camera2robotbase.coordinate_transform import CameraToRobotBaseConversion  

class PublishingSubscriber:

    def __init__(self):
        rospy.init_node('publishing_subscriber')
        self.subscriber = rospy.Subscriber('/pos_in_cam_frame', Float64MultiArray, self.pos_received, queue_size=10)
        self.publisher_pos_robot_frame = rospy.Publisher('/pos_in_robot_base_frame', Float64MultiArray, queue_size=10)

        x_disp = -17.8
        y_disp = 24.4
        z_disp = 0.0
        rot_angle = 180

        self.cam_to_robo = CameraToRobotBaseConversion(rot_angle, x_disp, y_disp, z_disp)

    def pos_received(self, msg):
        # callback function, when get the message of object position
        object_position = msg.data
        cam_ref_coord = np.array([
            [object_position[0]],
            [object_position[1]],
            [0.0],  # object_position[2] 
            [1]
        ])

        robot_base_frame_coord = self.cam_to_robo.convert(cam_ref_coord)  # Return robot base frame coordinates

        object_position = [robot_base_frame_coord[0][0], robot_base_frame_coord[1][0]] # robot_base_frame_coord[2][0]

        self.publish_position(object_position)

    def publish_position(self, object_position):
        msg = Float64MultiArray()
        msg.data = object_position
        self.publisher_pos_robot_frame.publish(msg)  # Publish the position to the topic

def main():
    publishing_subscriber = PublishingSubscriber()

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rospy.spin()  # Process one round of callbacks
        rate.sleep()

if __name__ == '__main__':
    main()
