#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose


def main():
    rospy.init_node("cam_gripper")
    
    rate = rospy.Rate(50)
    
    while rospy.is_shutdown():
        br = tf.TransformBroadcaster(queue_size=1)
        br.sendTransform(
            (-0.06, 0, 0.04),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "cam_co",
            "mani_co"
        )
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass