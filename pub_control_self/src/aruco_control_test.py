#!/usr/bin/env python
# -- coding: utf-8 --

from ast import Str
import rospy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose

class aruco_test():
    def __init__(self):
        self.pub_arucoid = rospy.Publisher("ARUCO_ID", Int32, queue_size=1)
        self.pub_arucop = rospy.Publisher("ARUCO_P", Pose, queue_size=1)
    
    def pub_aruco_id_1(self):
        foo_pose = Pose()
        self.pub_arucoid.publish(1)
        self.pub_arucop.publish(foo_pose)
    
    def pub_aruco_id_2(self):
        foo_pose = Pose()
        self.pub_arucoid.publish(2)
        self.pub_arucop.publish(foo_pose)


def main():
    rospy.init_node("aruco_control_test")
    foo = aruco_test()
    rate = rospy.Rate(1)
    
    while True:
        print(1)
        for i in range(3):
            rate.sleep()
            
        foo.pub_aruco_id_1()
        print("pub_1")
        
        
if __name__ == "__main__":
    main()