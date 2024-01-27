from __future__ import print_function
import numpy as np
import rospy
import tf
import tf2_ros
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from camera_robot_tf.calculate_transformmatrix import CalculateTransformMatrix

point_A1, point_A2, point_A3 = None, None, None
point_B1, point_B2, point_B3 = None, None, None
point_A,point_B = None, None

def camerapoint_callback(msg):
    global point_A
    point_A = np.array([msg.x, msg.y, msg.z])


def robotendpoint_callback(msg):
    global point_B
    point_B = np.array(msg.O_T_EE[12:15])


def robot_subscriber():
    rospy.init_node('transformation_init_robot', anonymous=True)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, robotendpoint_callback)
    rospy.spin()

def camera_subscriber():
    rospy.init_node('transformation_init_camera', anonymous=True)
    rospy.Subscriber('/matlabinit', Point, camerapoint_callback, queue_size=1)
    rospy.spin()

def calculate_and_apply_transformmatrix():
    Init = CalculateTransformMatrix()
    Init.point_A1 = point_A1
    Init.point_A2 = point_A2
    Init.point_A3 = point_A3
    Init.point_B1 = point_B1
    Init.point_B2 = point_B2
    Init.point_B3 = point_B3
    Init.cali_rotation()
    Init.cali_translation()
    return Init.m_rotation, Init.m_translation

def check_and_continue():
    input('Press Enter to confirm.')

def publish_tf(rotation, translation):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfs = TransformStamped()
    tfs.header.frame_id = 'camera'
    tfs.child_frame_id = 'robot'
    tfs.header.stamp = rospy.Time.now()
    tfs.transform.translation.x = translation[0]
    tfs.transform.translation.y = translation[1]
    tfs.transform.translation.z = translation[2]
    quaternion = tf.transformations.quaternion_from_matrix(rotation)
    tfs.transform.rotation.x = quaternion[0]
    tfs.transform.rotation.y = quaternion[1]
    tfs.transform.rotation.z = quaternion[2]
    tfs.transform.rotation.w = quaternion[3]

    broadcaster.sendTransform(tfs)
    rospy.spin()

def main():
    global point_A1, point_A2, point_A3, point_B1, point_B2, point_B3 ,point_A , point_B
    rospy.init_node('transformation_init', anonymous=True)
    rospy.Subscriber('/matlabinit', Point, camerapoint_callback, queue_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, robotendpoint_callback, queue_size=1)
    rospy.sleep(1)
    point_A1 = point_A
    point_B1 = point_B
    print('The point_A1 is:\n', point_A1)
    print('The point_B1 is:\n', point_B1)
    input('Press Enter to confirm the first point and get the next point.')
    point_A2 = point_A
    point_B2 = point_B
    print('The point_A2 is:\n', point_A2)
    print('The point_B2 is:\n', point_B2)
    input('Press Enter to confirm the second point and get the next point.')
    point_A3 = point_A
    point_B3 = point_B
    print('The point_A3 is:\n', point_A3)
    print('The point_B3 is:\n', point_B3)
    print('Now you can calculate the transformation matrix.')
    rotation, translation = calculate_and_apply_transformmatrix()
    print('The rotation matrix is:\n', rotation)
    print('The translation matrix is:\n', translation)
    publish_tf(rotation, translation)
    print('The transformation matrix is published.\n')


if __name__ == '__main__':
    main()

