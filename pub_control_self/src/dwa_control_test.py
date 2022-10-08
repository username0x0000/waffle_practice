#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String, Int32
    

class dwa_test:
    def __init__(self):
        self.pub_control = rospy.Publisher("DWA", String, queue_size=1)
        
    
    def pub_dwa_success(self):
        self.pub_control.publish("dwa_success")
    
    def pub_dwa_working(self):
        self.pub_control.publish("dwa_working")


def main():
    rospy.init_node("dwa_control_test")
    foo = dwa_test()
    rate = rospy.Rate(1)
    
    
    while True:
        for i in range(3):
            rate.sleep()
            
        foo.pub_dwa_success()
        print("dwa_success")


if __name__== "__main__":
    main()