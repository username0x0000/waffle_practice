#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose

class Trandfrom():
    def __init__(self):
        self.pose = Pose()
        self.br = tf.TransformBroadcaster(queue_size=1)
        self.listener = tf.TransformListener()
    
    def tf_aruco(self, object_pose):
        print("aruco_tf start")
        self.br.sendTransform(
            (object_pose.position.x, object_pose.position.y object_pose.position.z),
            (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w), 
            rospy.Time.now(),
            "object_co",
            "rgb_co"
        )
        self.br.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2),
            rospy.Time.now(),
            "rgb_co",
            "cam_co"
        )
        self.br.sendTransform(
            (-0.06, 0, 0.04),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "cam_co",
            "mani_co"
        )
        self.br.sendTransform(
            (object_pose.position.x, object_pose.position.y object_pose.position.z),
            (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w), 
            rospy.Time.now(),
            "gripper_co",
            "base_co"
        )
    
    def tf_mani(self, mani_pose):
        print("mani_tf start")
        pass
    
    def tf_dwa(self, dwa_pose):
        print("dwa_tf start")
        pass

def main():
    rospy.init_node("tf")
    tran = Trandfrom()
    
    rospy.Subscriber("aruco_tf", Pose, tran.tf_aruco)
    rospy.Subscriber("mani_tf", Pose, tran.tf_mani)
    rospy.Subscriber("dwa_tf", Pose, trans.tf_dwa)
    
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass