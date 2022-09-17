#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from gomtery_msgs.msg import Bool
from geometry_msgs.msg import Pose

class marker_to_rgb:
    def __init__(self):
        self.br = tf.TransformBroadcaster(queue_size=1)
    
    def broadcasting(self, marker_co):
        self.br.sendTransform(
            (marker_co.position.x, marker_co.position.y marker_co.position.z),
            (marker_co.orientation.x, marker_co.orientation.y, marker_co.orientation.z, marker_co.orientation.w), 
            rospy.Time.now(),
            "object_co",
            "rgb_co"
        )


def main():
    rospy.init_node("marker_rgb")
    
    tf = marker_to_rgb
    rospy.Subscrier("ARUCO_S", Pose, tf.broadcast)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass