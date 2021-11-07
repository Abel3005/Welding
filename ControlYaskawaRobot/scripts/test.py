#!/usr/bin/env python
#-*- coding: utf-8 -*-

from codecs import xmlcharrefreplace_errors
import sys
import os
import copy
import time
import numpy as np
from moveit_commander import move_group
import rospy
import serial # HANA ros_serial 패키지 의존성
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
import tf
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
import math

from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

depth_img_msg_name="/camera/aligned_depth_to_color/image_raw"


# bottom welding pose
QBOTTOM = quaternion_from_euler(-3.1415927, -1.395, 0.0)  # end-effector의 회전변환 (y축 기준 79도)
# left welding pose
QLEFT = quaternion_from_euler(-3.1415, -1.0707963, 0.707963)
# right welding pose
#QRIGHT = quaternion_from_euler(-3.1415, -1.0707963, -0.707963)
QRIGHT = - quaternion_from_euler(-3.1415, -1.0707963, -0.707963)

def goto_home_pose(move_group,wait=True):
    current_pose = move_group.get_current_pose().pose

    # 홈 포지션에서의 end-effector의 위치(base 기준)
    current_pose.position.x =  0.388350 #0.268350 #(m)
    current_pose.position.y =  0.0      #0.019599 #(m)
    current_pose.position.z =  0.801373 #0.861373 #(m) 

    q = quaternion_from_euler(-3.1415927, -1.1853982, 0.0)  # end-effector의 회전변환 (y축 기준 67도)
    current_pose.orientation.x = q[0] #0.97 #0.0
    current_pose.orientation.y = q[1] #0.0 #0.0
    current_pose.orientation.z = q[2] #0.23 #0.0
    current_pose.orientation.w = q[3] #0.0 #1.0

    move_group.set_pose_target(current_pose)
    move_group.go(wait=wait)
    move_group.stop()
    move_group.clear_pose_targets()

def rotate_joint(move_group,joint_index,value,wait=True):
    joint_goal = move_group.get_current_joint_values()

    joint_goal[joint_index] = value

    move_group.go(joint_goal,wait=wait)
    move_group.stop()

def goto_pose(move_group,x,y,z,torch_rotation = QBOTTOM,wait=True):
    current_pose = move_group.get_current_pose().pose

    current_pose.position.x = x
    current_pose.position.y = y
    current_pose.position.z = z

    q = torch_rotation
    
    current_pose.orientation.x = q[0]
    current_pose.orientation.y = q[1]
    current_pose.orientation.z = q[2] 
    current_pose.orientation.w = q[3] 

    move_group.set_pose_target(current_pose)
    move_group.go(wait=wait)
    move_group.stop()
    move_group.clear_pose_targets()

def goto_pose_linear(move_group,x,y,z,wait=True):
    waypoints = []
    current_pose = move_group.get_current_pose().pose
    waypoints.append(current_pose)

    goal_pose = copy.deepcopy(current_pose)

    goal_pose.position.x = x
    goal_pose.position.y = y
    goal_pose.position.z = z
    waypoints.append(goal_pose)

    plan,_ = move_group.compute_cartesian_path(waypoints,0.01,0)
    move_group.execute(plan,wait=wait)
    move_group.stop()
    #move_group.clear_pose_targets()

class ar1440:
    def __init__(self):
        pass


if __name__ == '__main__':
    ## 먼저 `moveit_commander` 와 `rospy` 노드를 초기화합니다:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)


    ## `RobotCommander` 객체를 생성합니다.
    ## 이 객체는 로봇의 기구학 모델과 현재의 관절 상태와 같은 정보를 제공합니다.
    robot = moveit_commander.RobotCommander()

    ## `PlanningSceneInterface` 객체를 생성합니다.
    ## 이 객체는 로봇의 주변 환경에 대한 정보를 획득, 설정, 갱신하는 인터페이스를 제공합니다.
    scene = moveit_commander.PlanningSceneInterface()

    ## `MoveGroupCommander` 객체를 생성합니다.
    ## 이 객체는 플래닝 그룹(관절 그룹)에 대한 인터페이스를 제공합니다.
    ## 여기서는 ar1440 로봇의 관절 그룹의 이름을 "manipulator"로 설정합니다.
    ## 이 인터페이스는 모션을 계획하고 실행하는 데 사용됩니다:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Rviz에서 궤적을 표시하는 데 사용되는 `DisplayTrajectory` ROS 퍼블리셔를 생성합니다.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## 기본 정보 획득#-*- coding: utf-8 -*-
    ## ^^^^^^^^^^^^^
    # 로봇에 대한 참조 프레임의 이름을 출력합니다
    planning_frame = move_group.get_planning_frame()
    #print("============ Planning frame: %s" % planning_frame)

    # 그룹에 대한 말단장치 링크의 이름을 출력합니다
    eef_link = move_group.get_end_effector_link()
    #print("============ End effector link: %s" % eef_link)

    # 로봇의 모든 그룹 목록을 출력합니다
    group_names = robot.get_group_names()
    #print("============ Available Planning Groups:", robot.get_group_names())
    
    # 로봇의 현재 상태를 출력합니다
    print("============ Printing robot state")
    print(robot.get_current_state())
    #print("")

                          
    os.system("rosservice call /robot_enable") # 로봇 서보전원 ON

    while True:
        cmd = raw_input('command : ')

        if cmd == 'home':
            goto_home_pose(move_group)

        elif cmd == 'rotate':
            index = int(input('which joint you want to rotate : '))
            value = float(input('value : '))
            rotate_joint(move_group,index,value)
        elif cmd == 'goto':
            x = float(input('x : '))
            y = float(input('y : '))
            z = float(input('z : '))
            r = raw_input('torch_position : ')

            if r == 'Bottom':
                goto_pose(move_group,x,y,z,QBOTTOM)
            elif r == 'Left':
                goto_pose(move_group,x,y,z,QLEFT)
            elif r == 'Right':
                goto_pose(move_group,x,y,z,QRIGHT)
        elif cmd == 'goto_linear':
            x = float(input('x : '))
            y = float(input('y : '))
            z = float(input('z : '))

            goto_pose_linear(move_group,x,y,z)
        elif cmd == 'info':
            info_cmd = raw_input('joint, position : ')
            
            if info_cmd == 'joint':
                joint_idx = 0
                
                for joint_value in move_group.get_current_joint_values():
                    print('%d : %f'%(joint_idx,joint_value))
                    joint_idx += 1

            elif info_cmd == 'position':
                current_pose = move_group.get_current_pose().pose
                print('position_x : %f'%(current_pose.position.x))
                print('position_y : %f'%(current_pose.position.y))
                print('position_z : %f'%(current_pose.position.z))
                print('orientation_x : %f'%(current_pose.orientation.x))
                print('orientation_y : %f'%(current_pose.orientation.w))
                print('orientation_z : %f'%(current_pose.orientation.z))
                print('orientation_w : %f'%(current_pose.orientation.w))
        elif cmd == 'exit':
            break