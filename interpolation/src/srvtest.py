#!/usr/bin/env python
import numpy as np
import cv2
import copy
import rospy
import tf
import csv
import keyboard
from collections import deque
from cv_bridge import CvBridge
from common import TFcfg, IPcfg
from TF_converter import TFcvt
from sensor_msgs.msg import Image
from interpolation.srv import *
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)


TF_cfg = TFcfg()
IP_cfg = IPcfg()
TF_cvt = TFcvt()
bridge = CvBridge()
depth_stack = deque([])
rgb_stack= deque([])
objp = np.zeros((IP_cfg.check_size,3), np.float32)
objp[:,:2] = np.mgrid[0:IP_cfg.check_height,0:IP_cfg.check_width].T.reshape(-1,2)
def append_rgb(msg):
    rgb_stack.append(bridge.imgmsg_to_cv2(msg))
    if len(rgb_stack) == 100:
        rgb_stack.popleft()   
    return
def append_depth(msg):
    depth_stack.append(bridge.imgmsg_to_cv2(msg))
    if len(depth_stack) == 100:
        depth_stack.popleft()

    return
def UV2RPoint(u,v,trans, rot):
    TF_cfg.img = depth_stack.pop()
    h_e_array = TF_cfg.h_e_array
    rotmat = TF_cvt.rotMat_from_quat(rot[0], rot[1], rot[2], rot[3])
    current_p = TF_cfg.get_robot_pose_mat(rotmat, trans)
    TF_cfg.TF_matrix = np.dot(current_p, h_e_array)
    #result = TF_cvt.uv2point(TF_cfg,u,v)
    s_x,s_y,s_z=UV2CPoint(u,v)
    result = TF_cvt.tfPoint(s_x,s_y,s_z,TF_cfg.TF_matrix)
    result[0] += 0.07 +0.03  - 0.09 + 0.001
    result[1] += 0.007 
    result[2] += -0.05 - 0.027 -0.14
    return result[0], result[1], result[2]
def UV2CPoint(u,v):
    try:
      uv_camera = rospy.ServiceProxy('/darknet_3d/uv_converter', uv2xyz)
      result = uv_camera(u,v)
      return result.x , result.y, result.z
    except rospy.ServiceException, e:
      print("Service call failed")
def add_two_ints_client(x,y):
   rospy.wait_for_service('add_two_ints')
   try:
      add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
      resp1 = add_two_ints(x,y)
      return resp1.sum
   except rospy.ServiceException, e:
      print "Service call failed: %s" %e

def main():
    rospy.init_node("CSV_Test")
    TF_cfg.set_tf_listener()
    rospy.Subscriber(TF_cfg.img_msg_name,Image,append_rgb)
    rospy.Subscriber(TF_cfg.depth_img_msg_name,Image,append_depth)
    rate = rospy.Rate(30)
    csvfile = None
    print('please input csvfile name')
    filename = raw_input()
    filename += '.csv'
    filelength = 0
    #select csvfile io mode
    print('please select csvfile mode ([0]: overwrite, [1]: followwrite, [other]: terminate')
    ans = raw_input()
    if int(ans) == 0:
        csvfile = open(filename, 'w')
    elif int(ans) == 1:
        tmpfile = open(filename, 'r')
        rdr = csv.reader(tmpfile)
        filelength = sum(1 for row in rdr)
        csvfile = open(filename, 'a')
    # else:
    #     return
    # wr = csv.writer(csvfile)
    #system manage
    flag = True
    #user message
    text_flag = True
    while not rospy.is_shutdown():
        #ground truth
        try:
            (trans, rot) = TF_cfg.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #img
        if len(rgb_stack) == 0:
            continue
        if len(depth_stack) == 0:
            continue
        if int(ans) == 2:
            i_u = input("u: ")
            i_v = input("v: ")
            result = UV2RPoint(i_u,i_v,trans,rot)
            print("x: %f"%(result[0]))
            print("y: %f"%(result[1]))
            print("z: %f"%(result[2]))
            print("g_x: %f"%(trans[0]))
            print("g_y: %f"%(trans[1]))
            print("g_z: %f"%(trans[2]))
            continue
        objpoints = []
        imgpoints = []
        bgr_image = rgb_stack.pop()
        gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_image, (IP_cfg.check_height,IP_cfg.check_width), None)
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray_image, corners, (5,5), (-1,-1), criteria)
            imgpoints.append(corners2)
            rgb_image = cv2.drawChessboardCorners(rgb_image, (IP_cfg.check_height,IP_cfg.check_width), corners2, ret)
            point_shape = corners2.shape
            if(text_flag == True): 
                print("enter 's' write current points")
                text_flag = False
            if keyboard.is_pressed('s') and flag:
                print('\n write %d' % (filelength+1))
                for i in range (0,point_shape[0]):
                    print("point[%d]: (%d,%d)"% (i+1,corners2[i][0][0],corners2[i][0][1] ))
                    x,y,z= UV2RPoint(int(corners2[i][0][0]), int(corners2[i][0][1]), trans, rot)
                    wr.writerow([filelength+(i+1), str(x[0]),str(y[0]),str(z[0])])
                flag = False
                print("enter 'c' continue, enter 'n' exit")
        if keyboard.is_pressed('c') and flag == False:
            print('\ninput mode')
            flag = True
            text_flag = True
        if keyboard.is_pressed('n'):
            break
            #print('corners_shape: ',corners2)
            #cv2.imshow('img',rgb_image)
            #cv2.waitKey(0)
        cv2.imshow('img',rgb_image)
        cv2.waitKey(1)
        rate.sleep()
    return
if __name__ == "__main__":
    main()
    





