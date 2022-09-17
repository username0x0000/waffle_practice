#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import sys, select, os, tty, termios


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    rospy.init_node('getKey')
    pub = rospy.Publisher("getKeyPose", Pose, queue_size=1)
    x, y, z = 0, 0, 0
    mode = 'x'
    n = 0
    cnt = 0
    while cnt < 5:
        try:
            while not rospy.is_shutdown():
                key = getKey()
                if key == 'x':
                    print("x : ")
                    mode = 'x'
                elif key == 'y':
                    print("y : ")
                    mode = 'y'
                elif key == "z":
                    print("z : ")
                    mode = 'z'
                else:
                    pass
                print("xyz")
                if int(key) in range(10):
                    n = int(key)
                else:
                    pass
                print("n")
                if mode == "x":
                    x = n
                elif mode == "y":
                    y = n
                elif mode == "z":
                    z = n
                else:
                    pass
                    print("n2xyz")
                print(mode, n)
                pose_n = Pose()
                pose_n.position.x = x
                pose_n.position.y = y
                pose_n.position.z = z
        except:
            print("error")
            cnt += 1
