#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Int32

# 수정이 필요한 부분 : patrol부분 dwa에게 랜덤 좌표 주는 것 필요
#                     pose 좌표 받아오는 것 추가 필요

mani_door_catch = 1
mani_door_release = 2
mani_object_catch = 3
mani_object_release = 4
mani_fail = 0
mani_success = 1


class control_tower:
    def __init__(self):
        # dwa, mani 관련 퍼블리시 dm = dwa mode, dp = dwa pose, mp = manipulator pose
        self.pub_dm = rospy.Publisher("dwa_m", String, queue_size=1)
        self.pub_dp = rospy.Publisher("dwa_p", Pose, queue_size=1)
        self.pub_mm = rospy.Publisher("mani_m", Int32, queue_size=1)
        self.pub_mp = rospy.Publisher("mani_p", Pose, queue_size=1)
        
        # tf listener 활성화 
        self.listener = tf.TransformListener()
        
        # pose = 현재 필요한 위치 좌표, foo_pose = mani에게 필요한 쓰레기 좌표값
        self.pose = Pose()
        self.foo_pose = Pose()
        
        # 컨트롤 모드
        self.mode = "patrol"
        self.control_status = "patrol"
        self.mode_list = {
            "patrol":self.patrol,
            "door":self.door,
            "object":self.object,
            "home":self.home
        }
        
        # aruco, dwa, manipulator status
        self.aruco_status = "need"
        self.dwa_status = " "
        self.mani_status = " "
        
        # 카메라의 진동에의한 흔들림 제거
        self.rate = rospy.Rate(1)


    def check_mode(self):
        print("(%s), (%s)\n(%s) (%s), (%s)"%(self.mode, self.control_status, self.dwa_status, self.mani_status, self.aruco_status))
        if self.dwa_status != "dwa_working" or self.mani_status != "mani_working" or self.control_status == "patrol":
            
            if self.control_status == "arrive":
                self.rate.sleep()
                self.rate.sleep()
            
            self.mode_list[self.mode]()

        
    def patrol(self):
        self.pub_dm.publish("patrol")
        self.init_status()
    
    def door(self):
        if self.control_status == "patrol":
            d_pose = self.pose
            d_pose.position.x -= 0.1
            self.pub_dp.publish(d_pose)
            if self.dwa_status == "dwa_success":
                self.control_status = "arrive"
            
        elif self.control_status == "arrive":
            self.aruco_status = "need"
            self.control_status = "catchable"
            
        elif self.control_status == "catchable" and self.aruco_status == "wait":
            self.pub_mm.publish(mani_door_catch)
            self.pub_mp.publish(self.pose)
            if self.mani_status == "mani_fail":
                self.control_status = "patrol"
                self.aruco_status = "need"
                return
            elif self.mani_status == "mani_success":
                self.control_status = "catch"
                return
            self.mani_status = "mani_working"
        
        elif self.control_status == "catch":
            self.open_door()
            self.control_status = "release"
        
        elif self.control_status == "release":
            self.pub_mm.publish(mani_door_release)
            self.pub_mp.publish(self.foo_pose)
            self.mani_status = "mani_working"
            self.control_status = "go inside"
        
        elif self.control_status == "go inside":
            self.go_inside()
            self.mode = "patrol"
    
    def object(self):
        if self.control_status == "patrol":
            d_pose = self.pose
            d_pose.position.x -= 0.1
            self.pub_dp.publish(d_pose)
            if self.dwa_status == "dwa_success":
                self.control_status = "arrive"
            
        elif self.control_status == "arrive":
            self.aruco_status = "need"
            self.control_status = "catchable"
            
        elif self.control_status == "catchable" and self.aruco_status == "wait":
            self.pub_mm.publish(mani_door_catch)
            self.pub_mp.publish(self.pose)
            if self.mani_status == "mani_fail":
                self.control_status = "patrol"
                self.aruco_status = "need"
                return
            elif self.mani_status == "mani_success":
                self.control_status = "catch"
                return
            self.mani_status = "mani_working"
                
        elif self.control_status == "catch":
            self.mode = "home"
            self.init_status()
            
    def home(self):
        if self.control_status == "patrol":
            self.pub_dm.publish("home")
            self.control_status = "home_arrived"
            
        elif self.control_status == "home_arrived":
            self.pub_mm(mani_object_release)
            self.pub_mp(self.foo_pose)
    
    def go_inside(self):
        self.pub_dm.publish("back_8")
    
    def open_door(self):
        self.pub_dm.publish("straight_7")

    def init_status(self):
        self.control_status = "patrol"
        self.aruco_status = "need"
        self.dwa_status = " "
        self.mani_status = " "
    
    def save_ap(self, a_pose):
        if self.aruco_status == "wait":
            return
        return ######################################################
        # base에서 marker까지의 좌표계 변환
        try:
            if self.control_status == "patrol":
                (trans, rot) = self.listener.lookupTransform("/map", "/object_co", rospy.Time(0))
            elif self.control_status == "catchable":
                (trans, rot) = self.listener.lookupTransform("/base_link", "/object_co", rospy.Time(0))
            print("tf success")
            print(self.mode, self.control_status, self.aruco_status)
            
            self.pose.position.x = trans[0]
            self.pose.position.y = trans[1]
            self.pose.position.z = trans[2]
            self.pose.orientation.x = rot[0]
            self.pose.orientation.y = rot[1]
            self.pose.orientation.z = rot[2]
            self.pose.orientation.w = rot[3]
            
            self.aruco_status = "wait"
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            print("except :", exc)

    def save_as(self, a_status):
        if self.aruco_status == "wait":
            return
        elif a_status.data == 1:
            self.mode = "door"
        elif a_status.data == 2:
            self.mode = "object"
        else:
            self.mode = "error"
            self.error = "save_as" + str(a_status.data)
            print(a_status.data)
            return
        self.aruco_status = "wait" ######################################################

    def save_ds(self, d_status):
        if d_status.data == "dwa_success":
            self.dwa_status = "dwa_success"
        elif d_status.data == "dwa_working":
            self.dwa_status.data = "dwa_working"

    def save_ms(self, m_status):
        if m_status.data == mani_success:
            self.mani_status = "mani_success"
        elif m_status.data == mani_fail:
            self.mani_status = "mani_fail"
        else:
            self.mani_status = "mani_working"

def main():
    rospy.init_node("control")
    
    control = control_tower()

    while True:
        rospy.Subscriber("MANI", Int32, control.save_ms)
        rospy.Subscriber("DWA", String, control.save_ds)
        rospy.Subscriber("ARUCO_ID", Int32, control.save_as)
        rospy.Subscriber("ARUCO_P", Pose, control.save_ap)
        control.check_mode()
    #rospy.spin()
    
        

if __name__ == "__main__": main()