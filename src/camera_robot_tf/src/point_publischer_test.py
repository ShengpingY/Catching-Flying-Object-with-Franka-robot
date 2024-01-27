#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

class PointPublisher(object):
    def __init__(self):
        rospy.init_node('point_publisher', anonymous=True)
        self.pub = rospy.Publisher('matlabinit', Point, queue_size=10)
        self.rate = rospy.Rate(1)
        
    def get_coordinates_of_object(self):
        while not rospy.is_shutdown():
            x = input("Enter x coordinate: ")
            y = input("Enter y coordinate: ")
            z = input("Enter z coordinate: ")

            try:
                x = float(x)
                y = float(y)
                z = float(z)
            except ValueError:
                rospy.logerr("Invalid input. Please enter a valid number.")
                continue

            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = z
            
            return point_msg

    def publish_coordinates(self, msg):
            self.pub.publish(msg)

def main():
    camera_publisher = PointPublisher()
    point = camera_publisher.get_coordinates_of_object()
    
    while not rospy.is_shutdown():
        camera_publisher.publish_coordinates(point)
        camera_publisher.rate.sleep()
        

if __name__ == '__main__':
    main()
