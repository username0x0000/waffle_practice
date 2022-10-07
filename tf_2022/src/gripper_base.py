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
        print(gripper_co.position.x, gripper_co.position.y, gripper_co.position.z)
        self.br.sendTransform(
            (gripper_co.position.x, gripper_co.position.y, gripper_co.position.z),
            (gripper_co.orientation.x, gripper_co.orientation.y, gripper_co.orientation.z, gripper_co.orientation.w), 
            rospy.Time.now(),
            "mani_co",
            "/base_link"
        )

def main():
    rospy.init_node("gripper_base")
    
    g2b = gripper_to_base()
    rospy.Subscriber("mani_pose", Pose, g2b.broadcasting)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass