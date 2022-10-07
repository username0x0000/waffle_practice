#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose

class Marker_to_rgb:
    def __init__(self):
        self.br = tf.TransformBroadcaster(queue_size=1)
    
    def broadcasting(self, marker_co):
        self.br.sendTransform(
            (marker_co.position.x, marker_co.position.y, marker_co.position.z),
            (tf.transformations.quaternion_from_euler(marker_co.orientation.x, marker_co.orientation.y, marker_co.orientation.z)), 
            rospy.Time.now(),
            "object_co",
            "rgb_co"
        )


def main():
    rospy.init_node("marker_rgb")
    
    m2r = Marker_to_rgb()
    rospy.Subscriber("ARUCO_P", Pose, m2r.broadcasting)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass