#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose


def main():
    rospy.init_node("rgb_cam")
    
    rate = rospy.Rate(100)
    
    br = tf.TransformBroadcaster(queue_size=1)
        
    while not rospy.is_shutdown():
        br.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2),
            rospy.Time.now(),
            "rgb_co",
            "cam_co"
        )
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass