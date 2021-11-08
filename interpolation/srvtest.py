#!/usr/bin/env python
import rospy
from interpolation.srv import *

def main():
    rospy.init_node("srv_test_node")
    print('srv_test node start')
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        print('a')
        rate.sleep()
    return
if __name__ == "__main__":
    main()