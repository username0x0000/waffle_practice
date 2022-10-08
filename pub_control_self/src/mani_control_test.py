#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String, Int32

mani_fail = 0
mani_success = 1
mani_working = 2

class mani_test:
    def __init__(self):
        self.pub_control = rospy.Publisher("MANI", Int32, queue_size=1)
    
    def pub_mani_success(self):
        self.pub_control.publish(mani_success)
    
    def pub_mani_fail(self):
        self.pub_control.publish(mani_fail)
    
    def pub_mani_working(self):
        self.pub_control.publish(mani_working)


def main():
    rospy.init_node("mani_control_test")
    foo = mani_test()
    rate = rospy.Rate(1)
    
    while True:
        for i in range(3):
            rate.sleep()
        
        foo.pub_mani_success()
        print("pub_mani")


if __name__ == "__main__":
    main()