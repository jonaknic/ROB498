import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped,TransformStamped
from std_srvs.srv import Empty, EmptyResponse
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import time
import math
from pyquaternion import Quaternion
from PCA import *

CUR_POSE= PoseStamped()
goal= PoseStamped()
TOL=0.3

def update_pose(msg):
    global CUR_POSE
    CUR_POSE = msg

def PosError(goal):
    global CUR_POSE
    dx=CUR_POSE.pose.position.x-goal.pose.position.x
    dy=CUR_POSE.pose.position.y-goal.pose.position.y
    dz=CUR_POSE.pose.position.z-goal.pose.position.z
    return math.sqrt(dx**2+dy**2+dz**2)

def planner(avoid_dir,WAYPOINTS,OBSTACLES):
    global CUR_POSE
    rate = rospy.Rate(20)
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = update_pose)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    p1 = np.array([CUR_POSE.pose.position.x,CUR_POSE.pose.position.y])
    p2 = np.array([WAYPOINTS[0],WAYPOINTS[1]])
    p3 = np.array([OBSTACLES[0]],OBSTACLES[1])
    obs_dis = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

    dx = WAYPOINTS[0] - CUR_POSE.pose.position.x
    dy = WAYPOINTS[1] - CUR_POSE.pose.position.y
    dz = WAYPOINTS[2] - CUR_POSE.pose.position.z
    theta = np.arctan(dx/dy)

    if WAYPOINTS[0] - CUR_POSE.pose.position.x < 0:
        x_dir = -1
    else:
        x_dir = 1
    if WAYPOINTS[1] - CUR_POSE.pose.position.y < 0:
        y_dir = -1
    else:
        y_dir = 1

    move = 0

    while move == 0:
        if avoid_dir == 1:
            # left
            p1_1 = np.array([CUR_POSE.pose.position.x + 1.2 * np.sin(theta) * (-y_dir),CUR_POSE.pose.position.y + 1.2 * np.cos(theta) * (x_dir)])
            p2_1 = np.array([CUR_POSE.pose.position.x + 1.2 * np.sin(theta) * (-y_dir) + dx, CUR_POSE.pose.position.y + 1.2 * np.cos(theta) * (x_dir) + dy])
            obs_dis_1 = np.linalg.norm(np.cross(p2_1-p1_1, p1_1-p3))/np.linalg.norm(p2_1-p1_1)
            if obs_dis_1 > 0.6:
                goal.pose.position.x = CUR_POSE.pose.position.x + 1.2 * np.sin(theta) * (-y_dir)
                goal.pose.position.y = CUR_POSE.pose.position.y + 1.2 * np.cos(theta) * (x_dir)
                goal.pose.position.z = CUR_POSE.pose.position.z
            else:
                goal.pose.position.x = CUR_POSE.pose.position.x + 1.8 * np.sin(theta) * (-y_dir)
                goal.pose.position.y = CUR_POSE.pose.position.y + 1.8 * np.cos(theta) * (x_dir)
                goal.pose.position.z = CUR_POSE.pose.position.z
        else:
            # right
            p1_1 = np.array([CUR_POSE.pose.position.x + 1.2 * np.sin(theta) * (y_dir),CUR_POSE.pose.position.y + 1.2 * np.cos(theta) * (-x_dir)])
            p2_1 = np.array([CUR_POSE.pose.position.x + 1.2 * np.sin(theta) * (y_dir) + dx, CUR_POSE.pose.position.y + 1.2 * np.cos(theta) * (-x_dir) + dy])
            obs_dis_1 = np.linalg.norm(np.cross(p2_1-p1_1, p1_1-p3))/np.linalg.norm(p2_1-p1_1)
            if obs_dis_1 > 0.6:
                goal.pose.position.x = CUR_POSE.pose.position.x + 1.2 * np.sin(theta) * (y_dir)
                goal.pose.position.y = CUR_POSE.pose.position.y + 1.2 * np.cos(theta) * (-x_dir)
                goal.pose.position.z = CUR_POSE.pose.position.z
            else:
                goal.pose.position.x = CUR_POSE.pose.position.x + 1.8 * np.sin(theta) * (y_dir)
                goal.pose.position.y = CUR_POSE.pose.position.y + 1.8 * np.cos(theta) * (-x_dir)
                goal.pose.position.z = CUR_POSE.pose.position.z
        local_pos_pub.publish(goal)
        if PosError(goal) < TOL:
            print("Reached first point: ",1)
            print(CUR_POSE.pose.position.x,CUR_POSE.pose.position.y,CUR_POSE.pose.position.z)
            move = 1
        rate.sleep()

    while move == 1:
        p1_2 = np.array([CUR_POSE.pose.position.x,CUR_POSE.pose.position.y])
        p2_2 = np.array([CUR_POSE.pose.position.x + dx,CUR_POSE.pose.position.y + dy])
        obs_dis_2 = np.linalg.norm(np.cross(p2_2-p1_2, p1_2-p3))/np.linalg.norm(p2_2-p1_2)
        if obs_dis_2 > 0.6:
            goal.pose.position.x = CUR_POSE.pose.position.x + dx
            goal.pose.position.y = CUR_POSE.pose.position.y + dy
            goal.pose.position.z = CUR_POSE.pose.position.z + dz
        else:
            if avoid_dir == 1:
                # left
                goal.pose.position.x = CUR_POSE.pose.position.x + dx + 0.6 * np.sin(theta) * (-y_dir)
                goal.pose.position.y = CUR_POSE.pose.position.y + dy + 0.6 * np.cos(theta) * (x_dir)
                goal.pose.position.z = CUR_POSE.pose.position.z + dz
            else: 
                # right
                goal.pose.position.x = CUR_POSE.pose.position.x + dx + 0.6 * np.sin(theta) * (y_dir)
                goal.pose.position.y = CUR_POSE.pose.position.y + dy + 0.6 * np.cos(theta) * (-x_dir)
                goal.pose.position.z = CUR_POSE.pose.position.z + dz
        local_pos_pub.publish(goal)
        if PosError(goal) < TOL:
            print("Reached first point: ",1)
            print(CUR_POSE.pose.position.x,CUR_POSE.pose.position.y,CUR_POSE.pose.position.z)
            move = 0
        rate.sleep()