#!/usr/bin/env python
# -- coding: utf-8 --

import re
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, String

class control_tower:
    def __init__(self):
        # dwa, mani 관련 퍼블리시 dm = dwa mode, dp = dwa pose, mp = manipulator pose
        self.pub_dm = rospy.Publisher("dwa_m", String, queue_size=1)
        self.pub_dp = rospy.Publisher("dwa_p", Pose, queue_size=1)
        self.pub_mp = rospy.Publisher("cur_mani_pose", Pose, queue_size=1)
        
        # tf listener 활성화 
        self.listener = tf.TransformListener()
        
        # 지금 해야하는 pose 데이터 저장
        self.pose = Pose()
        
        # 컨트롤 모드
        self.mode = "dwa_fin"
        
        # aruco, dwa, manipulator status
        self.aruco_status = "need"
        self.dwa_status = " "
        self.mani_status = " "
        
        self.rate = rospy.Rate(1)

    def check_mode(self):
        
        # print(self.mode) #########
        
        if self.mode == "init_program":
            self.pub_dm.publish("patrol")

        elif self.mode == "door_checked":
            print("door_checked")
            d_pose = Pose()
            d_pose.position.x = 1.658
            d_pose.position.y = 1.218
            self.pub_dp.publish(d_pose)
            self.aruco_status = "wait"
            
        elif self.mode == "dwa_fin":
            self.aruco_status = "need"
            
        elif self.mode == "grip":
            self.pub_mp.publish(self.pose)
            print("pub\n\n\n\n\n\n\n\n\n\n\n\n")
            self.pose = Pose()
            self.mode = "iaaaaa"
        
        ####### 현재 위치 받아오는 코드 필요 ##########
        elif self.mode == "mani_grip":
            d_pose = self.pose ##### 현재 위치
            d_pose.position.x -= 0.1
            self.pub_dm.publish("go")
            self.pub_dp.publish(d_pose)

        elif self.mode == "door_opened":
            self.pub_mr.publish("ready")
        
        elif self.mode == "object_checked":
            d_pose = self.pose
            d_pose.position.x -= 0.1
            print("x: ", d_pose.position.x, "y: ", d_pose.position.y, "z: ", d_pose.position.z) #########
            self.pub_dm.publish("go")
            self.pub_dp.publish(d_pose)
        
        elif self.mode == "go_home":
            ##### 홈의 위치로 재이동
            pass
    
    
    def check_aruco(self):
        if self.aruco_status == "need":
            return True
        else:
            return False

    def save_ap(self):
        # base에서 marker까지의 좌표계 변환
        print("save_ap")
        
        try:
            (trans, rot) = self.listener.lookupTransform("/base_footprint", "/object_co", rospy.Time(0))
            print("tf success")
            
            self.pose.position.x = trans[0]
            self.pose.position.y = trans[1]
            self.pose.position.z = trans[2]
            self.pose.orientation.x = rot[0]
            self.pose.orientation.y = rot[1]
            self.pose.orientation.z = rot[2]
            self.pose.orientation.w = rot[3]
            self.mode = "grip"
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("except")
            pass

        print("end")


    def save_as(self, a_status):
        self.aruco_status = a_status
        if self.aruco_status.data == 1:
            self.rate.sleep()
            # self.mode = "grip"
            # print("save_as", self.mode)
            # self.aruco_status = "wait"
            self.save_ap()
            
            print("jdksahgflkiujsagflius", self.mode)
        elif self.aruco_status.data == 2:
            self.mode = "object_checked"
            self.save_ap()
        else:
            pass

    def save_ds(self, d_status):
        self.dwa_status = d_status
        if self.dwa_status == "dwa_fin":
            self.mode = "dwa_fin"

    def save_ms(self, m_status):
        self.mani_status = m_status
        if self.mani_status == "mani_fin":
            self.mode = "mani_fin"

def main():
    rospy.init_node("test_control")
    
    control = control_tower()
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
        if control.check_aruco():
            sub_as = rospy.Subscriber("ARUCO_ID", Int32, control.save_as)
        sub_ds = rospy.Subscriber("DWA_S", String, control.save_ds)
        sub_ms = rospy.Subscriber("MANI_S", String, control.save_ms)
        
        control.check_mode()
        rate.sleep()
        
        if control.mode == "iaaaaa":
            return

    #rospy.spin()

if __name__ == "__main__": main()
