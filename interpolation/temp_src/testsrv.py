#!/usr/bin/env python
import numpy as np
from tf_test.srvUV2XYZ import *
import rospy
def main():
    rospy.init_node('srv_test')
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      # result =  UV2CPoint(1,2)
      print('a')
      rate.sleep()
    return
    
if __name__ == "__main__":
    main()