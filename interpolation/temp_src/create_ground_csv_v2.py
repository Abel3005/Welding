#!/usr/bin/env python 

import rospy
from create_ground_csv import tf_csv_write
from common import TFcfg
from interp_config import Interp_config
import csv
import pandas as pd
import copy
import sys
import numpy as np
import keyboard
import tf

ip_config = Interp_config()
tf_config = TFcfg()

def get_tf_position():
    #tf write
    try:
        (trans, rot) = tf_config.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    #if error apear, need exception code 
    return [trans[0], trans[1], trans[2]] 
    
def main():
    rospy.init_node('CSV_Ground_interpolation')
    rate = rospy.Rate(30)
    tf_config.set_tf_listener()
    print("enter output csv filename")
    output_filename = raw_input()
    output_filename += '.csv'
    print('print select mode(0: overwrite 1:Followwrite)')
    mode = input()
    file_length = None
    if(int(mode) == 0):
        with open(output_filename,'w') as csvfile:
            file_length = 0
            wr = csv.writer(csvfile)
            d_flag,b_flag,r_flag,flag, txtFlag = (False,False,False, True , True)
            down = []
            base = []
            right = []
            while not rospy.is_shutdown():
                if d_flag and b_flag and r_flag:
                    break
                if flag  and txtFlag:
                    print('[b]: write base_position')
                    print('[d]: write down_position')
                    print('[r]: write right_position')
                    txtFlag = False
                if keyboard.is_pressed('d') and flag:
                    down = get_tf_position()
                    print('entered down value')
                    flag = False
                    d_flag = True
                elif keyboard.is_pressed('b') and flag:
                    base = get_tf_position()
                    print('entered base value')
                    flag = False
                    b_flag = True
                elif keyboard.is_pressed('r') and flag:
                    right  = get_tf_position()
                    print('entered right value')
                    flag = False
                    r_flag = True
                elif keyboard.is_pressed('c'):
                    print('input mode')
                    flag = True
                    txtFlag = True
                rate.sleep()
            print('write csvfile')
            np_base = np.array(base)
            np_down = np.array(down)
            np_right = np.array(right)
            np_down_itp = np_down - np_base
            np_right_itp = np_right -np_base
            for i in range(ip_config.board_shape[1]):
                for j in range(ip_config.board_shape[0]):
                    wr.writerow([file_length,str(np_base[0]),str(np_base[1]),str(np_base[2])])
                    file_length += 1
                    np_base += np_down_itp
                np_base -= np_down_itp * ip_config.board_shape[0]
                np_base += np_right_itp

    elif(int(mode) == 1):
        df = pd.read_csv(output_filename)
        file_length = len(df)
        with open(output_filename,'a') as csvfile:
            wr = csv.writer(csvfile)
            wr = csv.writer(csvfile)
            d_flag,b_flag,r_flag,flag, txtFlag = (False,False,False, True , True)
            down = []
            base = []
            right = []
            while not rospy.is_shutdown():
                if d_flag and b_flag and r_flag:
                    break
                if flag  and txtFlag:
                    print('[b]: write base_position')
                    print('[d]: write down_position')
                    print('[r]: write right_position')
                    txtFlag = False
                if keyboard.is_pressed('d') and flag:
                    down = get_tf_position()
                    print('entered down value')
                    flag = False
                    d_flag = True
                elif keyboard.is_pressed('b') and flag:
                    base = get_tf_position()
                    print('entered base value')
                    flag = False
                    b_flag = True
                elif keyboard.is_pressed('r') and flag:
                    right  = get_tf_position()
                    print('entered right value')
                    flag = False
                    r_flag = True
                elif keyboard.is_pressed('c'):
                    print('input mode')
                    flag = True
                    txtFlag = True
                rate.sleep()
            print('write csvfile')
            np_base = np.array(base)
            np_down = np.array(down)
            np_right = np.array(right)
            np_down_itp = np_down - np_base
            np_right_itp = np_right -np_base
            for i in range(ip_config.board_shape[1]):
                for j in range(ip_config.board_shape[0]):
                    wr.writerow([file_length,str(np_base[0]),str(np_base[1]),str(np_base[2])])
                    file_length += 1
                    np_base += np_down_itp
                np_base -= np_down_itp * ip_config.board_shape[0]
                np_base += np_right_itp
    return

if __name__ == "__main__":
    main()