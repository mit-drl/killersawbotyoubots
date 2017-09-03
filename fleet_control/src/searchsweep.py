#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('fleet_control')
import rospy
import numpy as np
import openravepy as orpy
import re
import time
import copy
import tf.transformations as tr
from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Point,PoseStamped
from assembly_common.srv import BasePose

from youbotpy import YoubotEnv
import openravepy as orpy
from youbotpy import youbotik as yik

import matplotlib.pyplot as plt
import threading
import sys
from itertools import izip
from mit_msgs.msg import MocapPosition
from std_msgs.msg import Float64, Bool, String
from hole_detection.msg import Hole
from hole_detection.srv import HoleDetectionParametrization
from fleet_control.msg import VelTwist
from fleet_control.srv import SearchHole,SearchHoleResponse,InsertToHole,Switch,SwitchRequest

from vicon_utils import transform_by_subjects

import IPython
import tfplugin
from brics_actuator.msg import JointVelocities, JointValue, JointPositions

coarse_velocity = 0.006
#fine_velocity = 0.002
#fine_velocity = 0.015
fine_velocity = 0.005

simulation = True

position = np.array([0.0,0.0,0.0])
got_position = False
position_lock = threading.Lock()

hole = Hole()
hole.width = -1.0
hole.position.x = -1.0
hole.position.y = -1.0
hole.found = False
hole_lock = threading.Lock()

MAX_VEL = 0.4
THRESH = 0.003

arm_names = []
for i in range(5):
    arm_names.append('arm_joint_' + str(i + 1))
    
unit = 's^-1 rad'

all_robot_names = ['drc2']
r = all_robot_names[0]
envfile = 'environments/floor.env.xml'

offset = np.array([2.950, 1.1345, -2.5482, 1.7890, 2.9234])

def GetClosestArm(cur_arm, sol):
    best_dist = sys.maxint
    ret_i = 0
    for (i, v) in enumerate(sol):
        dist = np.linalg.norm(v - cur_arm, np.inf )
        if dist < best_dist:
            best_dist = dist
            ret_i = i
    print 'getting closest arm'
    #IPython.embed()
    return sol[ret_i]

def MoveArmTo(robot,goal,planner):
    try:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Verbose)
        with env:
            robot.SetActiveDOFs(range(5))
            traj = planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.01)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    finally:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Info)
        
def getEndEffector():
    return youbots[r].GetManipulators()[0].GetEndEffectorTransform()
    
def createVelocity(vels):
    for i in range(len(vels)):
        vels[i] = float(vels[i])
    vels = np.array(vels)
    #clamp
    if np.linalg.norm(vels, 2) > MAX_VEL:
        vels /= np.linalg.norm(vels, 2)
        vels *=  MAX_VEL
    
    v = JointVelocities()
    for i in range(5):
        v.velocities.append(JointValue())
        v.velocities[i].joint_uri = arm_names[i]
        v.velocities[i].unit = unit
        v.velocities[i].value = float(vels[i])
    return v
    
def stop():
    v = createVelocity([0, 0, 0, 0, 0])
    pub.publish(v)

def hole_found(msg): # This is a callback. Can execute in a separate thread.
    global hole
    global hole_lock
    hole_lock.acquire()
    hole.width = msg.width
    hole.position = msg.position
    hole.found = msg.found
    hole_lock.release()


def set_position(location): # This is a callback. Can execute in a separate thread.
    global position
    global got_position
    global position_lock
    position_lock.acquire()
    got_position = True
    position = np.array([location.linear.x, location.linear.y, location.angular.z])
    position_lock.release()


# move is the required motion as a tuple: (x,y,yaw)
def step(move, vel):
    goal = VelTwist()
    global position_lock
    position_lock.acquire()
    cur = copy.copy(position)
    position_lock.release()
    goal.target.linear.x = cur[0] + move[0]
    goal.target.linear.y = cur[1] + move[1]
    goal.target.angular.z = cur[2] + move[2]
    goal.velocity.data = vel
    pub_search.publish(goal)

def MoveRobotTo_robot_relative(pose,vel):
    global position_lock
    position_lock.acquire()
    cur = copy.copy(position)
    position_lock.release()
    pose_in_world = TransformPose(pose,cur)
    MoveRobotTo(pose_in_world,vel)

def MoveRobotTo(pose,vel):
    goal = VelTwist()
    goal.target.linear.x = pose[0]
    goal.target.linear.y = pose[1]
    goal.target.angular.z = pose[2]
    goal.velocity.data = vel
    pub_search.publish(goal)

def J1Search(minlimit,maxlimit,resolution):
    global youbotenv
    global myname_bare
    angles = youbotenv.GetRobot(myname_bare).GetDOFValues()[:5]
    nsteps = np.ceil((maxlimit-minlimit)/resolution)
    maxhole_angles = None
    maxhole = Hole()
    maxhole.width = -1.0
    maxhole.found = False
    for step in np.linspace(minlimit,maxlimit,nsteps):
        newangles = angles.copy()
        newangles[0] = newangles[0] + step
        youbotenv.MoveArm(myname_bare,newangles)
        time.sleep(0.3)
        hole = WaitForHole(0.,0.1)
        if hole is not None and hole.width > maxhole.width:
            print 'J1Search found hole of witdh: ',hole.width
            maxhole_angles = newangles
            maxhole = hole
    youbotenv.MoveArm(myname_bare,angles) # Move to original config.
    return maxhole_angles,maxhole

def SearchHoleWithArm(pathname):
    global youbotenv
    global myname_bare
    if pathname == 'ladder_hole_fine':
        plane_height = -0.235 #-0.241
    elif pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_fine_2_bottom':
        plane_height = -0.226
    elif pathname == 'ladder_hole_fine_4':
        plane_height = -0.205
    else:
        plane_height = -0.215
    hole_detection_parametrization(crop_window_x_min_param=-0.015, 
                                   crop_window_x_max_param=0.015,
                                   crop_window_y_min_param=-0.50,
                                   crop_window_y_max_param=-0.10,
                                   plane_height_param=plane_height,
                                   plane_uncertainty_param=0.025,
                                   hole_width_param=0.015,
                                   max_plane_angle_param=1.57,
                                   make_before_after_check_param=True,
                                   enforce_n_plane_points_before_hole_param=3,
                                   enforce_n_plane_points_after_hole_param=3,
                                   min_n_points_param=5)
    current_hole = None
    #angles,current_hole = J1Search(-0.1,0.1,0.005)
    #if angles is None:
    #    print 'first search returned none'
    #    return None
    #youbotenv.MoveArm(myname_bare,angles)
    #time.sleep(0.3)
    #align_speed = fine_velocity/2.0
    #MoveRobotTo_robot_relative((current_hole.position.x,0.0,0.0), align_speed)
    #time.sleep((abs(current_hole.position.x)/align_speed) + 1.0)
    #raw_input('hit enter for second alignment')
    lowerlimit = -0.008
    upperlimit =  0.008
    step = 0.00100
    ntries = 0
    while not rospy.is_shutdown(): 
        hole = WaitForHole(0.,0.3)
        if hole is not None and hole.width > 0.0095:
            break # we are done
        elif hole is None:
            pass
        #elif hole is None or hole.width <= 0.0:
        #    print 'hole not seen. very coarse search.'
        #    lowerlimit =-0.03
        #    upperlimit = 0.03
        #    step = 0.008 
        #elif hole.width <= 0.003:
        #    print 'hole barely seen. coarse search.'
        #    lowerlimit =-0.021
        #    upperlimit = 0.021
        #    step = 0.007 
        elif (pathname == 'ladder_hole_fine') and hole.width > 0.008:
            break # we are done for hole 1
        elif (pathname == 'ladder_hole_fine_4') and hole.width > 0.0055:
            break # we are done for hole 4
        elif (pathname == 'ladder_hole_fine_3') and hole.width > 0.0055:
            break # we are done for hole 3
        #elif hole.width <= 0.006:
        #    print 'hole seen but need much more width. fine search.'
        #    lowerlimit =-0.018
        #    upperlimit = 0.018
        #    step = 0.005 
        #elif (pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_fine_2_bottom') and hole.width > 0.007:
        elif (pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_fine_2_bottom') and hole.width > 0.003:
            break # we are done for hole 2,4
        #elif hole.width <= 0.009:
        #    print 'hole seen but need a little more width. very fine search.'
        #    #lowerlimit =-0.008
        #    #upperlimit = 0.008
        #    #step = 0.0015 
        #    lowerlimit =-0.012
        #    upperlimit = 0.012
        #    step = 0.003 
        #    #lowerlimit =-0.015
        #    #upperlimit = 0.015
        #    #step = 0.005 
        ntries += 1
        if ntries > 2:
            print 'hole search failed'
            return None
        angles,current_hole = J1Search(lowerlimit,upperlimit,step)
        if angles is not None:
            youbotenv.MoveArm(myname_bare,angles)
            #raw_input('hit enter to continue')
            time.sleep(1.5)
    return hole

def WaitForHole(goal_width,timeout):
    global hole_lock
    start_time = time.time()
    while not rospy.is_shutdown():
        hole_lock.acquire()
        current_hole = copy.copy(hole)
        hole_lock.release()
        if current_hole.width >= goal_width:
            return current_hole
        if (time.time()-start_time) >= timeout:
            return None

def GetPath(pathname,reduction=1.0):
    if pathname == 'ladder_hole' or pathname == 'ladder_hole_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_3' or pathname == 'ladder_hole_4':
        return LadderHoleSearchPath(right_limit=reduction*0.02,left_limit=reduction*0.02,up_limit=reduction*0.001,down_limit=reduction*0.001)
    elif pathname == 'ladder_hole_fine' or pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_fine_2_bottom' or pathname == 'ladder_hole_fine_3' or pathname == 'ladder_hole_fine_4':
        return LadderHoleSearchPath(right_limit=reduction*0.005,left_limit=reduction*0.005,up_limit=reduction*0.001,down_limit=reduction*0.001)
    elif pathname == 'skin_hole':
        return SkinHoleSearchPath2(right_limit=reduction*0.02,left_limit=reduction*0.02,up_limit=reduction*0.005,down_limit=reduction*0.005,nsteps=6)
    elif pathname == 'align' or pathname == 'prealign':
        return SkinHoleSearchPath2(right_limit=reduction*0.01,left_limit=reduction*0.01,up_limit=reduction*0.005,down_limit=reduction*0.005, nsteps=2)
    elif pathname == 'align2' or pathname == 'prealign2':
        return SkinHoleSearchPath2(right_limit=reduction*0.004,left_limit=reduction*0.004,up_limit=reduction*0.004,down_limit=reduction*0.004, nsteps=2)
    elif pathname == 'skin_hole2':
        return SecondSkinHoleSearchPath()
    else:
        print 'Error: unknown path name %s.'%(pathname)
        return None

def Search(pathname):
    print 'Searching %s.'%(pathname)
    global youbotenv
    global myname_bare
    if pathname == 'ladder_hole':
        service_switch(SwitchRequest(skin=False)) # Move drc4
        hole_detection_parametrization(crop_window_x_min_param=-0.017, 
                                       crop_window_x_max_param=0.017,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       #plane_height_param=-0.241,
                                       plane_height_param=-0.235,
                                       plane_uncertainty_param=0.025,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=1.57,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=3,
                                       enforce_n_plane_points_after_hole_param=3,
                                       min_n_points_param=5)
    elif pathname == 'ladder_hole_3':
        service_switch(SwitchRequest(skin=False)) # Move drc4
        hole_detection_parametrization(crop_window_x_min_param=-0.017, 
                                       crop_window_x_max_param=0.017,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       plane_height_param=-0.215,
                                       plane_uncertainty_param=0.025,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=1.57,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=3,
                                       enforce_n_plane_points_after_hole_param=3,
                                       min_n_points_param=5)
    elif pathname == 'ladder_hole_2_bottom':
        service_switch(SwitchRequest(skin=False)) # Move drc4
        hole_detection_parametrization(crop_window_x_min_param=-0.017, 
                                       crop_window_x_max_param=0.017,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       plane_height_param=-0.231,
                                       plane_uncertainty_param=0.025,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=1.57,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=3,
                                       enforce_n_plane_points_after_hole_param=3,
                                       min_n_points_param=5)
    elif pathname == 'ladder_hole_2':
        service_switch(SwitchRequest(skin=False)) # Move drc4
        hole_detection_parametrization(crop_window_x_min_param=-0.017, 
                                       crop_window_x_max_param=0.017,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       plane_height_param=-0.226,
                                       plane_uncertainty_param=0.025,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=1.57,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=3,
                                       enforce_n_plane_points_after_hole_param=3,
                                       min_n_points_param=5)
    elif pathname == 'ladder_hole_4':
        service_switch(SwitchRequest(skin=False)) # Move drc4
        hole_detection_parametrization(crop_window_x_min_param=-0.017, 
                                       crop_window_x_max_param=0.017,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       #plane_height_param=-0.215,
                                       plane_height_param=-0.205,
                                       plane_uncertainty_param=0.025,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=1.57,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=3,
                                       enforce_n_plane_points_after_hole_param=3,
                                       min_n_points_param=5)
    elif pathname == 'skin_hole':
        service_switch(SwitchRequest(skin=True)) # Move fleet
        hole_detection_parametrization(crop_window_x_min_param=-0.07,
                                       crop_window_x_max_param=0.07,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       #plane_height_param=-0.182,
                                       plane_height_param=-0.176,
                                       plane_uncertainty_param=0.03,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=0.75,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=5,
                                       enforce_n_plane_points_after_hole_param=10,
                                       min_n_points_param=20)
    elif pathname == 'skin_hole2':
        service_switch(SwitchRequest(skin=True)) # Move fleet
        hole_detection_parametrization(crop_window_x_min_param=-0.07,
                                       crop_window_x_max_param=0.07,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       #plane_height_param=-0.215,
                                       plane_height_param=-0.209,
                                       plane_uncertainty_param=0.025,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=0.75,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=5,
                                       enforce_n_plane_points_after_hole_param=10,
                                       min_n_points_param=20)
    elif pathname == 'align' or pathname == 'prealign':
        service_switch(SwitchRequest(skin=True)) # Move fleet
        hole_detection_parametrization(crop_window_x_min_param=-0.07, 
                                       crop_window_x_max_param=0.07,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       #plane_height_param=-0.215,
                                       plane_height_param=-0.209,
                                       plane_uncertainty_param=0.03,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=0.75,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=5,
                                       enforce_n_plane_points_after_hole_param=10,
                                       min_n_points_param=20)
    elif pathname == 'align2' or pathname == 'prealign2':
        service_switch(SwitchRequest(skin=True)) # Move fleet
        hole_detection_parametrization(crop_window_x_min_param=-0.07, 
                                       crop_window_x_max_param=0.07,
                                       crop_window_y_min_param=-0.50,
                                       crop_window_y_max_param=-0.10,
                                       #plane_height_param=-0.225,
                                       plane_height_param=-0.219,
                                       plane_uncertainty_param=0.03,
                                       hole_width_param=0.015,
                                       max_plane_angle_param=0.75,
                                       make_before_after_check_param=True,
                                       enforce_n_plane_points_before_hole_param=5,
                                       enforce_n_plane_points_after_hole_param=10,
                                       min_n_points_param=20)


    else:
        print 'Unknown pathname: %s'%(pathname)
    time.sleep(0.5)

    
    if pathname == 'ladder_hole_fine' or pathname == 'ladder_hole' or pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_fine_2_bottom' or pathname == 'ladder_hole_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_3' or pathname == 'ladder_hole_fine_3' or pathname == 'ladder_hole_4' or pathname == 'ladder_hole_fine_4':
        trytimes = 2
    elif pathname == 'align' or pathname == 'prealign':
        trytimes = 1
    elif pathname == 'align2' or pathname == 'prealign2':
        trytimes = 1
    else:
        trytimes = 4
    for i in range(trytimes):
        print 'trying searching %d th time.'%(i)
        global position_lock
        position_lock.acquire()
        start_pose = copy.copy(position)
        position_lock.release()
        robot_relative_path = GetPath(pathname,1.0/(i+1))
        path = TransformPath(robot_relative_path,start_pose)
        prev_pt = start_pose
        current_hole = None
        for pt in path: 
            if ('ladder_hole_fine' in pathname) or pathname == 'align2' or pathname == 'prealign2':
                vel = fine_velocity/1.0
            elif pathname == 'align' or pathname == 'prealign':
                vel = fine_velocity/1.0
            else:
                vel = fine_velocity
            MoveRobotTo(pt,vel)
            dist = np.linalg.norm((pt-prev_pt)[:2]) # ignoring rotation for now.
            timeout = (dist/vel)+1.0 # wait an extra second
            if pathname == 'align':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout)
            elif pathname == 'ladder_hole':
                current_hole = WaitForHole(goal_width=0.005, timeout=timeout)
            elif pathname == 'ladder_hole_fine':
                current_hole = WaitForHole(goal_width=0.0075, timeout=timeout)
            elif pathname == 'ladder_hole_3' or pathname == 'ladder_hole_fine_3':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout) #TODO: fix this?
            elif pathname == 'ladder_hole_2' or pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_fine_2_bottom':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout) #TODO: fix this?
            elif pathname == 'ladder_hole_4' or pathname == 'ladder_hole_fine_4':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout) #TODO: fix this?
            elif pathname == 'prealign2':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout)
            elif pathname == 'prealign':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout)
            elif pathname == 'align2':
                current_hole = WaitForHole(goal_width=0.003, timeout=timeout)
            else:
                current_hole = WaitForHole(goal_width=0.0035, timeout=timeout)
                print 'in the else'
            if current_hole is not None: # found the hole
                position_lock.acquire()
                cur_position = copy.copy(position)
                position_lock.release()
                MoveRobotTo(cur_position,fine_velocity) # stop where the robot is now.
                break
            prev_pt = pt
        if current_hole is None:
            print 'Hole not found.' 
            return None
        else:
            print 'Hole found. Centering under hokuyo.'
            # move hole under sensor
            #align_speed = fine_velocity/3.0
            align_speed = fine_velocity
            j0angle = -1.0 * youbotenv.GetRobot(myname_bare).GetDOFValues()[0]
            print 'j0angle: ',j0angle
            if pathname == 'ladder_hole' or pathname=='ladder_hole_fine' or pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_fine_2_bottom' or pathname == 'ladder_hole_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_3' or pathname == 'ladder_hole_fine_3' or pathname == 'ladder_hole_4' or pathname == 'ladder_hole_fine_4':
                MoveRobotTo_robot_relative((np.cos(j0angle)*current_hole.position.x,np.sin(j0angle)*current_hole.position.x,0.0), align_speed)
                #MoveRobotTo_robot_relative((current_hole.position.x,0.0,0.0), align_speed)
                time.sleep((abs(current_hole.position.x)/align_speed) + 3.0)
            elif pathname == 'skin_hole' or pathname == 'skin_hole2' or pathname == 'align' or pathname == 'align2' or pathname == 'prealign' or pathname == 'prealign2':
                MoveRobotTo_robot_relative((-1.0*(np.cos(j0angle)*current_hole.position.x),-1.0*(np.sin(j0angle)*current_hole.position.x),0.0), align_speed)
                #MoveRobotTo_robot_relative((-1.0*current_hole.position.x,0.0,0.0), align_speed)
                time.sleep((abs(current_hole.position.x)/align_speed) + 2.0)
                #MoveRobotTo_robot_relative((-1.0*(np.cos(j0angle)*current_hole.position.x/10.0),-1.0*(np.sin(j0angle)*current_hole.position.x/10.0),0.0), align_speed)
                #time.sleep((abs(current_hole.position.x/10.0)/align_speed) + 1.0)

                # MEHMET ADDED THIS TO TEST.
                current_hole = WaitForHole(goal_width=0.0035, timeout=4.0)
                if not (current_hole is None) and abs(current_hole.position.x) > 0.0015:
                    print pathname,': move in x did not perfectly align. current_hole.position.x: ',current_hole.position.x
                while not (current_hole is None) and abs(current_hole.position.x) > 0.0015:
                    MoveRobotTo_robot_relative((-1.0*(np.cos(j0angle)*current_hole.position.x),-1.0*(np.sin(j0angle)*current_hole.position.x),0.0), align_speed)
                    time.sleep((abs(current_hole.position.x)/align_speed) + 2.0)
                    current_hole = WaitForHole(goal_width=0.003, timeout=4.0)
                if not (current_hole is None):
                    print pathname,': move in x aligned! current_hole.position.x: ',current_hole.position.x
                else:
                    print pathname,': hole is lost during x alignment. new.'
                    continue

            if pathname == 'ladder_hole' or pathname == 'ladder_hole_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_3' or pathname == 'ladder_hole_4':
                current_hole = WaitForHole(goal_width=0.003, timeout=4.0)
                if not (current_hole is None) and abs(current_hole.position.x) > 0.0015:
                    print pathname,': move in x did not perfectly align. current_hole.position.x: ',current_hole.position.x
                while not (current_hole is None) and abs(current_hole.position.x) > 0.0015:
                    MoveRobotTo_robot_relative((np.cos(j0angle)*current_hole.position.x,np.sin(j0angle)*current_hole.position.x,0.0), align_speed)
                    time.sleep((abs(current_hole.position.x)/align_speed) + 2.0)
                    current_hole = WaitForHole(goal_width=0.003, timeout=4.0)
                # The arm search will take care of the rest. REMOVE IF NOT USING ARM SEARCH!!!!
                if not (current_hole is None):
                    print pathname,': move in x aligned! current_hole.position.x: ',current_hole.position.x
                    current_hole.position.x=0.0 # assuming we made it.
                else:
                    print pathname,': hole is lost during x alignment. new.'
                    continue
                return current_hole

            if pathname == 'align':
                current_hole = WaitForHole(goal_width=0.005, timeout=4.0)
            elif pathname == 'ladder_hole':
                current_hole = WaitForHole(goal_width=0.0045, timeout=4.0)
            elif pathname == 'ladder_hole_fine':
                current_hole = WaitForHole(goal_width=0.0045, timeout=4.0)
            elif pathname == 'ladder_hole_2' or pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_fine_2_bottom':
                current_hole = WaitForHole(goal_width=0.003, timeout=4.0) #TODO: fix this?
            elif pathname == 'ladder_hole_3' or pathname == 'ladder_hole_fine_3':
                current_hole = WaitForHole(goal_width=0.003, timeout=4.0) #TODO: fix this?
            elif pathname == 'ladder_hole_4' or pathname == 'ladder_hole_fine_4':
                current_hole = WaitForHole(goal_width=0.003, timeout=4.0) #TODO: fix this?
            elif pathname == 'align2':
                current_hole = WaitForHole(goal_width=0.0050, timeout=4.0)
            elif pathname == 'prealign2':
                current_hole = WaitForHole(goal_width=0.0050, timeout=4.0)
            elif pathname == 'prealign':
                current_hole = WaitForHole(goal_width=0.0050, timeout=4.0)
            else:
                print 'in the else'
                current_hole = WaitForHole(goal_width=0.0035, timeout=4.0)

            if current_hole is not None: # found the hole
                print 'Hole found! Returning.'
                return current_hole
            else:
                if pathname == 'ladder_hole_fine_2' or pathname == 'ladder_hole_2' or pathname == 'ladder_hole_fine_2_bottom' or pathname == 'ladder_hole_2_bottom' or pathname == 'ladder_hole_3' or pathname == 'ladder_hole_fine_3' or pathname == 'ladder_hole_4' or pathname == 'ladder_hole_fine_4':
                    return None
                print 'Hole was found but got lost while bringing under sensor.'
                #print 'Re-searching.'
                #return None # Or maybe just assume the hole is under sensor?
    return None


def XYYawToMatrix(xyyaw):
    mat = np.eye(3)
    ca = np.cos(xyyaw[2])
    sa = np.sin(xyyaw[2])
    mat[0,2] = xyyaw[0]
    mat[1,2] = xyyaw[1]
    mat[0,0] = ca
    mat[1,0] = sa
    mat[0,1] = -sa
    mat[1,1] = ca
    return mat


def MatrixToXYYaw(mat):
    return np.array([mat[0,2],mat[1,2],np.arctan2(mat[1,0],mat[0,0])])


def TransformPose(pose_in_robot, robot_in_world):
    robot_in_w_mat = XYYawToMatrix(robot_in_world)
    pose_in_r_mat = XYYawToMatrix(pose_in_robot)
    pose_in_world_mat = np.dot(robot_in_w_mat,pose_in_r_mat)
    return MatrixToXYYaw(pose_in_world_mat)

    
def TransformPath(path,start_pose): 
    new_path = []
    for pose in path:
        new_path.append(TransformPose(pose,start_pose))
    return new_path


def LadderHoleSearchPath(right_limit=0.02,left_limit=0.02,up_limit=0.01,down_limit=0.01,nsteps=1):
    path = []
    for i in range(nsteps):
        # go right (-y in robot frame)
        pt = np.array([0.0,-1.0*right_limit,0.0])
        path.append(pt)
        # go left 
        pt = np.array([0.0,1.0*left_limit,0.0])
        path.append(pt)
        # go up (+x in robot frame)
        pt = np.array([1.0*up_limit*(i+1),1.0*left_limit,0.0])
        path.append(pt)
        # go right
        pt = np.array([1.0*up_limit*(i+1),-1.0*right_limit,0.0])
        path.append(pt)
        # go down
        pt = np.array([-1.0*down_limit*(i+1),-1.0*right_limit,0.0])
        path.append(pt)
        # go left
        pt = np.array([-1.0*down_limit*(i+1),1.0*left_limit,0.0])
        path.append(pt)
        # go back to starting pose
        pt = np.array([0.0,0.0,0.0])
        path.append(pt)
    return path

def SkinHoleSearchPath2(right_limit=0.02,left_limit=0.02,up_limit=0.01,down_limit=0.01,nsteps=1):
    path = []
    pt = np.array([0.0,0.0,0.0])
    path.append(pt)
    for i in range(nsteps):
        # go left 
        pt = np.array([0.0,1.0*left_limit*(i+1),0.0])
        path.append(pt)
        # go right (-y in robot frame)
        pt = np.array([0.0,-1.0*right_limit*(i+1),0.0])
        path.append(pt)
        # go up (+x in robot frame)
        pt = np.array([1.0*up_limit*(i+1),-1.0*right_limit*(i+1),0.0])
        path.append(pt)
        # go left
        pt = np.array([1.0*up_limit*(i+1),1.0*left_limit*(i+1),0.0])
        path.append(pt)
        # go down
        pt = np.array([-1.0*down_limit*(i+1),1.0*left_limit*(i+1),0.0])
        path.append(pt)
        # go left
        pt = np.array([-1.0*down_limit*(i+1),-1.0*right_limit*(i+1),0.0])
        path.append(pt)
        # go back to starting pose
        pt = np.array([0.0,0.0,0.0])
        path.append(pt)
    return path



def SkinHoleSearchPath(right_limit=0.04,left_limit=0.0,up_step=0.01,n_steps=4):
    path = []
    for i in range(n_steps):
        # go right (-y in robot frame)
        pt = np.array([i*up_step,-1.0*right_limit,0.0])
        path.append(pt)
        # go up and left
        pt = np.array([(i+1)*up_step,left_limit,0.0])
        path.append(pt)
    return path


def AlignmentSearchPath():
    path = []
    # go right (-y in robot frame)
    pt = np.array([0.0,-0.01,0.0])
    path.append(pt)
    # go left
    pt = np.array([0.0,0.01,0.0])
    path.append(pt)
    # go center
    pt = np.array([0.0,0.0,0.0])
    path.append(pt)
    # go down
    pt = np.array([-0.01,0.0,0.0])
    path.append(pt)
    # go go up
    pt = np.array([0.01,0.0,0.0])
    path.append(pt)
    # go center
    pt = np.array([0.0,0.0,0.0])
    path.append(pt)
    return path


def SecondSkinHoleSearchPath():
    pass # TODO Later. But will probably be an arc.


def DrawPath(path):
    #hl, = plt.plot([], [])
    xs = []
    ys = []
    for pt in path:
        xs.append(pt[0])
        ys.append(pt[1])
        hl = plt.plot(xs,ys)
        plt.show(block=False)
        time.sleep(1.0)
        plt.close()
    #     hl.set_xdata(np.append(hl.get_xdata(), pt[0]))
    #     hl.set_ydata(np.append(hl.get_ydata(), pt[1]))
    #     plt.draw()
    #     plt.show()
    #     time.sleep(0.5)
    hl = plt.plot(xs,ys)
    plt.show()

def MoveEEStraight(velocity_factor,target,step):
    #vel_factor = 1.5
    while (target[2, 3] - getEndEffector()[2, 3]) * np.sign(step) > THRESH:
        current_arm = robot.GetDOFValues()[0:5]
        sub_target_z = getEndEffector()[2, 3] + step
        sub_target = copy.deepcopy(target)
        sub_target[2, 3] = sub_target_z
        sol = yik.FindIKSolutions(robot, sub_target)
        if sol is None or len(sol) == 0:
            break
        closest_arm = GetClosestArm(current_arm, sol)
        diff = closest_arm - current_arm
        v = createVelocity(velocity_factor*diff)
        v.velocities[0].value = 0.0
        pub.publish(v)
        rospy.sleep(0.1)
    stop()

def MoveClecoDownVel(velocity_factor,length=0.08):
    transform = getEndEffector()
    target = copy.deepcopy(transform)
    target[2, 3] -= length
    MoveEEStraight(velocity_factor,target,-0.005)

def MoveClecoUpVel(velocity_factor,length=0.08):
    transform = getEndEffector()
    target = copy.deepcopy(transform)
    target[2, 3] += length
    MoveEEStraight(velocity_factor,target,+0.005)

def MoveJ4(j4val):
    jp = JointPositions()
    jp.positions.append(JointValue())
    jp.positions[0].joint_uri = 'arm_joint_4'
    jp.positions[0].unit = 'rad'
    jp.positions[0].value = j4val
    arm_pos_pub.publish(jp)
    time.sleep(0.1)

def insert_wrapper(req):
    print 'In insert_wrapper'
    if req.hole_name == 'hole_1':
        #hole1_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -0.307,   1.400, 0.375, 0.0])
        #hole1_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.70338827e-01,   1.43593791e+00,  4.02400919e-01,  -1.53409523e-09])
        hole1_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.93040986e-01,   1.44751072e+00, 4.13530272e-01,  -1.53409509e-09])
        youbotenv.MoveArm(robot.GetName(),hole1_inital_config)
        time.sleep(2.0)
        MoveClecoDownVel(6.0,0.04)
        MoveClecoDownVel(5.0,0.085)
        time.sleep(1.0)
        #MoveRobotTo_robot_relative((-0.03,0.0,0.0), 0.15)
        newconfig = robot.GetDOFValues()[:5]
        #newconfig[3] -= 0.1
        #youbotenv.MoveArm(myname_bare,newconfig)
        MoveJ4(newconfig[3]-0.10)
        time.sleep(0.2)
        MoveClecoDownVel(5.0,0.015)
        time.sleep(0.5)

        ## Moving arm left/right for fastener magnet contact.
        #current_config = robot.GetDOFValues()[:5]
        #current_config_left = current_config.copy()
        #current_config_left[0] -= 0.006
        #current_config_right = current_config.copy()
        #current_config_right[0] += 0.006
        #youbotenv.MoveArm(robot.GetName(),current_config_left)
        #time.sleep(1.0)
        #youbotenv.MoveArm(robot.GetName(),current_config_right)
        #time.sleep(1.0)
    elif req.hole_name == 'hole_2':
        #hole2_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -0.307,   1.400, 0.375, 0.0])
        #hole2_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.70338827e-01,   1.43593791e+00,  4.02400919e-01,  -1.53409523e-09])
        #hole2_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.81711864e-01,   1.44182287e+00, 4.07888999e-01,  -1.53409516e-09])
        hole2_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.93040986e-01,   1.44751072e+00, 4.13530272e-01,  -1.53409509e-09])
        youbotenv.MoveArm(robot.GetName(),hole2_inital_config)
        time.sleep(2.0)
        MoveClecoDownVel(6.0,0.04)
        MoveClecoDownVel(5.0,0.08)
        time.sleep(1.0)
        #MoveRobotTo_robot_relative((-0.03,0.0,0.0), 0.15)
        newconfig = robot.GetDOFValues()[:5]
        #newconfig[3] -= 0.1
        #youbotenv.MoveArm(myname_bare,newconfig)
        MoveJ4(newconfig[3]-0.10)
        time.sleep(0.2)
        MoveClecoDownVel(5.0,0.015)
        time.sleep(0.5)
        ## Moving arm left/right for fastener magnet contact.
        #current_config = robot.GetDOFValues()[:5]
        #current_config_left = current_config.copy()
        #current_config_left[0] -= 0.006
        #current_config_right = current_config.copy()
        #current_config_right[0] += 0.006
        #youbotenv.MoveArm(robot.GetName(),current_config_left)
        #time.sleep(1.0)
        #youbotenv.MoveArm(robot.GetName(),current_config_right)
        #time.sleep(1.0)
    elif req.hole_name == 'hole_3':
        #hole3_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -0.307,   1.400, 0.375, 0.0])
        #hole3_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.70338827e-01,   1.43593791e+00,  4.02400919e-01,  -1.53409523e-09])
        #hole3_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.81711864e-01,   1.44182287e+00, 4.07888999e-01,  -1.53409516e-09])
        hole3_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.93040986e-01,   1.44751072e+00, 4.13530272e-01,  -1.53409509e-09])
        youbotenv.MoveArm(robot.GetName(),hole3_inital_config)
        time.sleep(2.0)
        MoveClecoDownVel(6.0,0.03)
        MoveClecoDownVel(5.0,0.073)
        time.sleep(1.0)
    elif req.hole_name == 'hole_4':
        #hole4_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -0.307,   1.400, 0.375, 0.0])
        #hole4_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.70338827e-01,   1.43593791e+00,  4.02400919e-01,  -1.53409523e-09])
        #hole4_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.81711864e-01,   1.44182287e+00, 4.07888999e-01,  -1.53409516e-09])
        hole4_inital_config = np.array([robot.GetDOFValues()[0]-0.004, -3.93040986e-01,   1.44751072e+00, 4.13530272e-01,  -1.53409509e-09])
        youbotenv.MoveArm(robot.GetName(),hole4_inital_config)
        time.sleep(2.0)
        MoveClecoDownVel(6.0,0.03)
        MoveClecoDownVel(5.0,0.073)
        time.sleep(1.0)
    return True

def lifttool_wrapper(req):
    print 'In lifttool_wrapper'
    if req.hole_name == 'hole_1':
        MoveClecoUpVel(6.0,0.115)
        time.sleep(0.5)
        youbotenv.MoveArm(myname_bare,hole_search_config) 
        #youbotenv.MoveArm(myname_bare,hole_search_config) 
        time.sleep(1.5)
    elif req.hole_name == 'hole_2':
        MoveClecoUpVel(6.0,0.115)
        time.sleep(0.5)
        youbotenv.MoveArm(myname_bare,hole_search_config) 
        #youbotenv.MoveArm(myname_bare,hole_search_config) 
        time.sleep(1.5)
    elif req.hole_name == 'hole_3':
        MoveClecoUpVel(6.0,0.10)
        time.sleep(0.5)
        youbotenv.MoveArm(myname_bare,hole_search_config) 
        time.sleep(1.5)
    elif req.hole_name == 'hole_4':
        MoveClecoUpVel(6.0,0.10)
        time.sleep(0.5)
        youbotenv.MoveArm(myname_bare,hole_search_config) 
        time.sleep(1.5)
    return True

def search_wrapper(req):
    global youbotenv
    global myname_bare
    global hole_search_config
    #current_hole = Search(req.hole_name)
    if 'ladder_hole_fine' == req.hole_name or 'ladder_hole_fine_2' == req.hole_name or 'ladder_hole_fine_2_bottom' == req.hole_name or 'ladder_hole_fine_3' == req.hole_name or 'ladder_hole_fine_4' == req.hole_name:
        current_hole = SearchHoleWithArm(req.hole_name)
    else:
        if req.hole_name == 'ladder_hole':#  or req.hole_name == 'ladder_hole_4':
            youbotenv.MoveArm(myname_bare,hole_search_config) 
            time.sleep(2.0)
        if req.hole_name == 'ladder_hole_2' or req.hole_name == 'ladder_hole_2_bottom' or req.hole_name == 'ladder_hole_3' or req.hole_name == 'ladder_hole_4':
            youbotenv.MoveArm(myname_bare,hole_search_config) 
            time.sleep(2.0)
        current_hole = Search(req.hole_name)
    if current_hole is None:
        if req.hole_name == 'ladder_hole':#  or req.hole_name == 'ladder_hole_4':
            youbotenv.MoveArm(myname_bare,hole_search_config)
            time.sleep(1.0)
        if req.hole_name == 'ladder_hole_2' or req.hole_name == 'ladder_hole_2_bottom' or req.hole_name == 'ladder_hole_3' or req.hole_name == 'ladder_hole_4':
            youbotenv.MoveArm(myname_bare,hole_search_config)
            time.sleep(1.0)
        return SearchHoleResponse(hole=Hole(found=False,width=-1.0,position=Point(x=-1.0,y=-1.0,z=-1.0)))
    return current_hole



if __name__ == "__main__":

    rospy.init_node('search')
    my_name = rospy.get_namespace()
    if my_name is '/':
        my_name = '/drc2/'

    rospy.Subscriber(my_name + 'location', Twist, set_position)
    service_switch = rospy.ServiceProxy('/switch', Switch)
    if not simulation:
        # wait for a position message
        while not rospy.is_shutdown():
            try:
                position_lock.acquire()
                if got_position:
                    break
            finally:
                position_lock.release()
 
    pub_search = rospy.Publisher(my_name+'ik_command_fine', VelTwist)
    rospy.Subscriber('/drc2/hole_found', Hole, hole_found)

    search_s = rospy.Service(my_name + 'start_search', SearchHole, search_wrapper)
    insert_s = rospy.Service(my_name + 'insert', InsertToHole, insert_wrapper)
    lifttool_s = rospy.Service(my_name + 'lifttool', InsertToHole, lifttool_wrapper)
    
    rospy.wait_for_service('/drc2/hole_detection/parametrize')
    try:
        hole_detection_parametrization = rospy.ServiceProxy('/drc2/hole_detection/parametrize', HoleDetectionParametrization)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    myname_bare = re.sub('/','',my_name) # strip off the slashes.
    youbotenv = YoubotEnv(sim=False,viewer=True,env_xml=envfile,youbot_names=[myname_bare])
    robot = youbotenv.youbots[myname_bare]
    yik.init(robot)
    env = youbotenv.env
    youbots = youbotenv.youbots

    pub = rospy.Publisher('/drc2/arm_1/arm_controller/velocity_command', JointVelocities, queue_size=1)
    arm_pos_pub = rospy.Publisher('/drc2/arm_1/arm_controller/position_command', JointPositions, latch=True)

    time.sleep(2.0)
    hole_search_config = np.array([ -3.26627433e-06,   4.40581116e-01,   2.25353629e-01, 0.88,   6.92751936e-06])

    youbotenv.MoveArm(myname_bare,hole_search_config)
    time.sleep(1.0)

    #TODO: don't hardcode this in the future
    arm_feedback = rospy.ServiceProxy('/drc2/arm_feedback_command', BasePose)

    rospy.spin()

