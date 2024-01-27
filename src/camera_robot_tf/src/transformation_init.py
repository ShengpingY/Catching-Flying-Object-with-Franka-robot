from __future__ import print_function
import rospy
import tf
import tf2_ros
import numpy as np
import geometry_msgs.msg
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Point
from camera_robot_tf.calculate_transformmatrix import CalculateTransformMatrix

class TransformationInit(object):
    def __init__(self):
        self.Init = CalculateTransformMatrix()
        self.camera_count = 0
        self.robot_count = 0
        rospy.init_node('transformation_init', anonymous=True)

        rospy.Subscriber('/matlabinti', Point, self.camerapoint_callback)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.robotendpoint_callback)

    def camerapoint_callback(self, msg):
        if self.camera_count == 0:
            self.Init.point_A1 = np.array([msg.x, msg.y, msg.z])
        elif self.camera_count == 1:
            self.Init.point_A2 = np.array([msg.x, msg.y, msg.z])
        elif self.camera_count == 2:
            self.Init.point_A3 = np.array([msg.x, msg.y, msg.z])

        self.camera_count += 1
        self.check_and_continue()

    def robotendpoint_callback(self, msg):
        if self.robot_count == 0:
            self.Init.point_B1 = msg.O_T_EE[12:15]
        elif self.robot_count == 1:
            self.Init.point_B2 = msg.O_T_EE[12:15]
        elif self.robot_count == 2:
            self.Init.point_B3 = msg.O_T_EE[12:15]

        self.robot_count += 1
        self.check_and_continue()

    def check_and_continue(self):
        input('Press Enter to confirm.')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TransformationInit()
        node.run()
    except rospy.ROSInterruptException:
        pass

    
