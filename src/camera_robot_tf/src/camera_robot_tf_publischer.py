from __future__ import print_function
import numpy as np
import rospy
import tf
import tf2_ros
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from camera_robot_tf.calculate_transformmatrix import CalculateTransformMatrix

count = 0
point_A1, point_A2, point_A3 = None, None, None
point_B1, point_B2, point_B3 = None, None, None

def camerapoint_callback(msg):
    global point
    point = np.array([msg.x, msg.y, msg.z])


# def camerapoint_callback(msg):
#     global count, point_A1, point_A2, point_A3
#     print('get the camera point position is:\n', msg)
#     while True:
#         count = input('Please input the number of the CAMERA point:\n')
#         if count in ['1', '2', '3']:
#             count = int(count)
#             break
#         else:
#             print('Please input a valid number (1, 2, or 3).\n')

#     if count == 1:
#         if point_A1 is not None:
#             print('The point A1 has been set. Please input another number.\n')
#         else:
#             point_A1 = np.array([msg.x, msg.y, msg.z])
#             print('point_A1 is :\n', point_A1)
#     elif count == 2:
#         if point_A2 is not None:
#             print('The point A2 has been set. Please input another number.\n')
#         else:
#             point_A2 = np.array([msg.x, msg.y, msg.z])
#             print('point_A2 is :\n', point_A2)
#     elif count == 3:
#         if point_A3 is not None:
#             print('The point A3 has been set. Please input another number.\n')
#         else:
#             point_A3 = np.array([msg.x, msg.y, msg.z])
#             print('point_A3 is :\n', point_A3)


def robotendpoint_callback(msg):
    global count, point_B1, point_B2, point_B3
    print('get the robot endpoint position is:\n', msg.O_T_EE[12:15])
    while True:
        count = input('Please input the number of the Robot endpoint:\n')
        if count in ['1', '2', '3']:
            count = int(count)
            break
        else:
            print('Please input a valid number (1, 2, or 3).\n')

    if count == 1:
        if point_B1 is not None:
            print('The point B1 has been set. Please input another number.\n')
        else:
            point_B1 = np.array(msg.O_T_EE[12:15])
            print('point_B1 is :\n', point_B1)
    elif count == 2:
        if point_B2 is not None:
            print('The point B2 has been set. Please input another number.\n')
        else:
            point_B2 = np.array(msg.O_T_EE[12:15])
            print('point_B2 is :\n', point_B2)
    elif count == 3:
        if point_B3 is not None:
            print('The point B3 has been set. Please input another number.\n')
        else:
            point_B3 = np.array(msg.O_T_EE[12:15])
            print('point_B3 is :\n', point_B3)


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
    ros.spin()

def main():
    rospy.init_node('transformation_init', anonymous=True)

    # 订阅相机和机器人的消息
    rospy.Subscriber('/matlabinit', Point, camerapoint_callback, queue_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, robotendpoint_callback, queue_size=1)


    while not rospy.is_shutdown():
        # if point_A1 is not None and point_A2 is not None and point_A3 is not None \
        #         and point_B1 is not None and point_B2 is not None and point_B3 is not None:

        # move to point A
        point_A1 = point
        # move to point B
        point_A2 = point
        # move to point C
        point_A3 = point
        
        #     print('Now you can calculate the transformation matrix.')
        #     rotation, translation = calculate_and_apply_transformmatrix()
        #     print('The rotation matrix is:', rotation)
        #     print('The translation matrix is:', translation)
        #     publish_tf(rotation, translation)
        #     print('The transformation matrix is published.')
        #     # rospy.sleep(0.1) # 如果收集到足够的数据


        


if __name__ == '__main__':
    main()
