#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("dwa_fin")
    pub = rospy.Publisher("DWA", String, queue_size=1)
    while True:
        pub.publish("dwa_success")
        

if __name__ == "__main__":
    main()