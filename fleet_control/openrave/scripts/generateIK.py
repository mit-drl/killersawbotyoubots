#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import openravepy as orpy
import time

import IPython
ipshell = IPython.embed 


def convert_to_real_youbot_joint_values(q):
    jointdiff = np.array([2.949606435870417,
                          1.1344640137963142,
                         -2.548180707911721,
                         1.7889624832941877,
                         2.923426497090502])
    return jointdiff + q

np.set_printoptions(suppress=True)
np.set_printoptions(precision=4)

env = orpy.Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('robots/kuka-youbot.robot.xml') 
#env.Load('/home/youbot/youbotmodel2/brsu-youBot-T/standalone/robots/youbot_5D_base_gripper.robot.xml') 
#env.Load('robots/kuka-youbot.zae') 
#env.SetDebugLevel(orpy.DebugLevel.Debug);
#ipshell()
robot = env.GetRobots()[0]
manip = robot.GetManipulators()[0]

# Load the IK database.
with robot.GetEnv():
    robot.SetActiveManipulator(manip)
    manip.ik_database = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.TranslationDirection5D)
    if not manip.ik_database.load():
        print 'Generating IK database for %s.'%manip.GetName()
        manip.ik_database.autogenerate()

target = np.array([ 0.42,  0,  0.13])
#target = np.array([ 0.39,  -0.06220808,  0.2342928])
#target = np.array([ 0.39,  0.0,  0.2342928])
direction = np.array([-1,0,0])
zstep = 0.001
prevtarget = None

# move up to find highest point with an ik.
while True:
    solution = manip.FindIKSolution(orpy.IkParameterization(orpy.Ray(target,direction),orpy.IkParameterization.Type.TranslationDirection5D),orpy.IkFilterOptions.CheckEnvCollisions)
    if solution is None:
        print 'highest: ',prevtarget
        target = prevtarget.copy()
        break
    else: 
        prevtarget = target.copy()
        target[2] = target[2] + zstep

path = []
while True:
    solution = manip.FindIKSolution(orpy.IkParameterization(orpy.Ray(target,direction),orpy.IkParameterization.Type.TranslationDirection5D),orpy.IkFilterOptions.CheckEnvCollisions)
    if solution is None:
        print 'lowest: ',prevtarget
        break
    else: 
        path.append(solution)
        prevtarget = target.copy()
        target[2] = target[2] - zstep

for q in path:
    robot.SetDOFValues(q,manip.GetArmIndices())
    print convert_to_real_youbot_joint_values(q)
    time.sleep(0.2)

