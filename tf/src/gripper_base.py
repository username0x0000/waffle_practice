#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose


class gripper_to_base:
    def __init__(self):
        self.br = tf.TransformBroadcaster(queue_size=1)
    
    def broadcasting(self, gripper_co):
        self.br.sendTransform(
            (gripper_co.position.x, gripper_co.position.y gripper_co.position.z),
            (gripper_co.orientation.x, gripper_co.orientation.y, gripper_co.orientation.z, gripper_co.orientation.w), 
            rospy.Time.now(),
            "object_co",
            "rgb_co"
        )

def main():
    rospy.init_node("gripper_base")
    
    tf = gripper_to_base
    rospy.Subscrier("control_mani", Pose, tf.broadcast)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass