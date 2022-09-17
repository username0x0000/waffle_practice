import rospy
import tf
from geomtery_msgs.msg import Pose
from std_msgs.msg import String

class control_tower:
    def __init__(self):
        # dwa, mani 관련 퍼블리시 dm = dwa mode, dp = dwa pose, mp = manipulator pose
        self.pub_dm = rospy.Publisher("dwa_m", String, queue_size=1)
        self.pub_dp = rospy.Publisher("dwa_p", Pose, queue_size=1)
        self.pub_mp = rospy.Publisher("mani_p", Pose, queue_size=1)
        
        # tf listener 활성화 
        listener = tf.TransformListener()
        
        # tf 활성화 퍼블리시
        self.marker2rgb = rospy.Publisher("marker2rgb", Pose, queue_size=1)
        self.rgb2cam = rospy.Publisher("rgb2cam", Pose, queue_size=1)
        self.cam2gripper = rospy.Publisher("cam2gripper", Pose, queue_size=1)
        self.gripper2base = rospy.Publisher("gripper2base", Pose, queue_size=1)
        
        # 지금 해야하는 pose 데이터 저장
        self.pose = Pose()
        
        # 컨트롤 모드
        self.control_status = "init_program"
        
        # aruco, dwa, manipulator status
        self.aruco_status = "need"
        self.dwa_status = " "
        self.mani_status = " "

    def check_mode(self):
        
        print(self.mode) #########
        
        if self.mode == "init_program":
            self.pub_dm.publish("patrol")

        elif self.mode == "door_checked":
            d_pose = self.pose
            d_pose.position.x -= 0.1
            self.pub_dm.publish("go")
            self.pub_dp.publish(d_pose)
            self.aruco_status = "wait"
            print("x: ", d_pose.position.x, "y: ", d_pose.position.y, "z: ", d_pose.position.z) #########
            
        elif self.mode == "dwa_fin":
            self.pub_mr.publish("ready")
            if self.mani_status == "ready":
                self.aruco_status = "need"
                m_pose = self.pose_for_mani()
                self.pub_mp.publish(m_pose)
                self.mani_status = "unready"
        
        ####### 현재 위치 받아오는 코드 필요 ##########
        elif self.mode == "mani_grip":
            d_pose = self.pose ##### 현재 위치
            d_pose.position.x -= 0.1
            self.pub_dm.publish("go")
            self.pub_dp.publish(d_pose)

        elif self.mode == "door_opened":
            self.pub_mr.publish("ready")
            if self.mani_status == "ready":
                self.aruco_status = "need"
                m_pose = self.pose_for_mani()
                self.pub_mp.publish(m_pose)
        
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

    def save_ap(self, a_pose):
        self.pose = a_pose
        pub = rospy.Publisher("control_aruco", Pose, queue_size=1)
        pub.publish(self.pose)
        (trans,rot) = self.listener.lookupTransform("/base", "object_co", rospy.Time(0))

    def save_as(self, a_status):
        self.aruco_status = a_status
        if self.aruco_status == "door":
            self.mode = "door_checked"
        elif self.aruco_status == "object":
            self.mode = "object_checked"
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
    
    control = control_tower
    if control.check_aruco():
        sub_as = rospy.Subscriber("ARUCO_S", String, control.save_as)
    sub_ds = rospy.Subscriber("DWA_S", String, control.save_ds)
    sub_ms = rospy.Subscriber("MANI_S", String, control.save_ms)
    
    control.check_mode()
    
    rospy.spin()

if __name__ == "__main__": main()