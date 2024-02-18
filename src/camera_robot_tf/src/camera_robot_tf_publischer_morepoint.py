from __future__ import print_function
import numpy as np
import rospy
# import tf
# import tf2_ros
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from camera_robot_tf.calculate_transformmatrix_morepoint import CalculateTransformMatrix

point_A,point_B = None, None
Array_Point_A = []
Array_Point_B = []

def camerapoint_callback(msg):
    global point_A
    point_A = np.array([msg.x, msg.y, msg.z])


def robotendpoint_callback(msg):
    global point_B
    point_B = np.array([msg.x, msg.y, msg.z])

# def tranformed_coordinate_pub():
#     rate = rospy.Rate(100) # 10hz
#     while not rospy.is_shutdown():
#         pub.publish(robotPoint)
#         rate.sleep()

def get_point(transMatrix):
    global point_A
    point_C = np.concatenate((point_A, np.array([1.0])), axis=0)
    transformed_point = np.dot(transMatrix, point_C.T)[:3]
    return transformed_point

def calculate_and_apply_transformmatrix():
    Init = CalculateTransformMatrix()
    for i in range(len(Array_Point_A)):
        Init.add_point_pair(Array_Point_A[i], Array_Point_B[i])
    Init.calibrate()
    transformation_Matrix = Init.get_transformmatrix()
    return transformation_Matrix

def main():
    global Array_Point_A, Array_Point_B
    rospy.init_node('transformation_init', anonymous=True)
    rospy.Subscriber('/matlab', Point, camerapoint_callback, queue_size=1)
    rospy.Subscriber('/state', Point, robotendpoint_callback, queue_size=1)
    pub = rospy.Publisher('/transformed_coord', Point, queue_size=1)
    rospy.sleep(1)

    # Collecting points for both camera and robot
    print("Collecting points for both camera and robot...")

    while not rospy.is_shutdown():
        # Continue collecting points until enough pairs are collected
        if point_A is not None and point_B is not None:
            Array_Point_A.append(point_A)
            Array_Point_B.append(point_B)
            print("Added point pair: ", point_A, point_B)
            input('Press Enter to confirm the point and go to the next point.')
            point_A, point_B = None, None
            rospy.sleep(1)  # Add a small delay to avoid multiple pairs in a short time

        if len(Array_Point_A) >= 5 and len(Array_Point_B) >= 5:
            break

    print('Collected 5 points for both camera and robot.')

    # Calculate and apply the transformation matrix
    transMatrix = calculate_and_apply_transformmatrix()
    print('The transformation matrix is:\n', transMatrix)

    # Publish transformed coordinates
    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        # Publish transformed coordinates
        robotPoint = get_point(transMatrix)
        point_msg = Point()
        point_msg.x = robotPoint[0]
        point_msg.y = robotPoint[1]
        point_msg.z = robotPoint[2]
        pub.publish(point_msg)
        rate.sleep()

if __name__ == '__main__':
    main()