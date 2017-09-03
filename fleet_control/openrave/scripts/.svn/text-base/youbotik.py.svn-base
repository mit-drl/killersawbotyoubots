#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import openravepy as orpy
import time
from itertools import izip

import IPython
ipshell = IPython.embed 

PALM_DIRECTION_IN_EE_FRAME = np.array([0.,0.,1.,0.]) # Palm is +z for current youbot openrave robot.

def PoseFromXYAndYaw(youbot,pt2d,yaw):
    pose = np.eye(4)
    pose[:2,3] = pt2d
    pose[2,3] = youbot.GetTransform()[2,3] # use current height of robot.
    pose[:3,:3] = orpy.rotationMatrixFromAxisAngle(np.array([0.,0.,1.])*yaw)
    return pose

def DiscretizeLineAroundPoint(point, direction, extent, resolution):
    norm = np.linalg.norm(direction) 
    if norm < 0.99999 or norm > 1.00001:
        print 'direction needs to be a unit vector.'
        raise Exception('direction needs to be a unit vector')
    points = []
    nsamples_per_direction = int(np.ceil(extent/resolution))
    points.append(point)
    for i in range(1,nsamples_per_direction+1):
        points.append(point-direction*i*resolution)
        points.append(point+direction*i*resolution)
    return points

def DiscretizeAnglesAroundDirection(direction, extent, resolution):
    if extent > np.pi:
        print 'Warning: rotation extent is truncated to pi.'
        extent = np.pi
    angles = [0]
    angles.extend(np.linspace(resolution,extent,int(np.floor(extent/resolution))))
    angles.extend(-1.0*np.linspace(resolution,extent,int(np.floor(extent/resolution))))
    offset = np.arctan2(direction[1],direction[0])
    return angles+np.tile(offset,len(angles))

def DiscretizeCircleAroundDirection(center, direction, radius, extent, resolution):
    angles = DiscretizeAnglesAroundDirection(direction, extent, resolution)
    points = []
    for angle in angles:
        pt = center + radius * (np.array([np.cos(angle), np.sin(angle)]))
        points.append(pt)
    return points,angles

def GetArmLink0Positions(eepose, maindirection, palm_looking_up, translationextent, translationresolution, rotationextent, rotationresolution):
    eepose = np.array(eepose)
    norm = np.linalg.norm(palm_direction_in_world_xy)
    if norm == 0: 
        # This means the palm is looking up. All search directions will work. Use current robot direction as anchor.
        positions = []
        for angle in DiscretizeAnglesAroundDirection(youbot.GetTransform()[:2,0], rotationextent,rotationresolution):
            axis = np.array([np.cos(angle), np.sin(angle)])
            newpositions = DiscretizeLineAroundPoint(eepose[:2,3],axis,extent=translationextent,resolution=translationresolution)
            positions.extend(newpositions)
    else:
        # Meaningful to search only along the palm direction.
        axis = palm_direction_in_world_xy/norm
        positions = DiscretizeLineAroundPoint(eepose[:2,3],axis,extent=translationextent,resolution=translationresolution)
    return positions

# position is in 2d (x,y). Use current z of robot.
# direction is in 2d (x,y).
def GetBasePosesGivenArmLink0Pose(youbot,armlink0position,armlink0direction,j0resolution):
    current_basepose = youbot.GetLinks()[0].GetTransform()
    current_armlink0pose = youbot.GetLinks()[1].GetTransform()
    base_to_armlink0_planar_dist = np.linalg.norm(current_basepose[:2,3]-current_armlink0pose[:2,3])
    extent=min(abs(youbot.GetJoints()[0].GetLimits()[0][0]),abs(youbot.GetJoints()[0].GetLimits()[1][0])), # Assuming almost symmetric min and max limits for j0.
    base_pts,neg_base_angles = DiscretizeCircleAroundDirection(center=armlink0position,
                                                               direction=-1.0*armlink0direction,
                                                               radius=base_to_armlink0_planar_dist,
                                                               extent=extent,
                                                               resolution=j0resolution)
    base_angles = -1.0*neg_base_angles
    return base_pts,base_angles

def FindIKSolutions(youbot, eepose, returnfirst=False, checkenvcollision=True, 
                    translationextent=1.0, translationresolution=0.01, rotationextent=np.pi, rotationresolution=0.05):
    manip = youbot.GetManipulators()[0]
    palmdirection = np.dot(eepose,PALM_DIRECTION_IN_EE_FRAME)[:2]
    norm = np.linalg.norm(palmdirection)
    arm_link0_positions = []
    base_directions_with_j0_set_to_0 = []
    if norm == 0:
        # This means the palm is looking up. A singular case where all search directions will work. Use current robot direction as main direction.
        for angle in DiscretizeAnglesAroundDirection(youbot.GetTransform()[:2,0],rotationextent,rotationresolution):
            axis = np.array([np.cos(angle), np.sin(angle)])
            newpositions = DiscretizeLineAroundPoint(eepose[:2,3],axis,extent=translationextent,resolution=translationresolution)
            arm_link0_positions.extend(newpositions)
            base_directions_with_j0_set_to_0.extend([axis]*len(newpositions))
    else:
        arm_link0_positions = DiscretizeLineAroundPoint(eepose[:2,3],palmdirection/norm,extent=translationextent,resolution=translationresolution)
        base_directions_with_j0_set_to_0 = [palmdirection/norm]*len(arm_link0_positions)
    base_poses = []
    arm_configs = []
    for position,direction_when_j0_is_0 in izip(armlink0_positions,base_directions_with_j0_set_to_0):
        base_pts,base_yaws = GetBasePosesGivenArmLink0Pose(youbot,position,direction_when_j0_is_0,j0resolution=rotationresolution)
        with youbot:
            youbot.SetTransform(PoseFromXYAndYaw(youbot,base_pts[0],base_yaws[0]))
            soln = manip.FindIKSolution(orpy.IkParameterization(orpy.Ray(eepose[:3,3], # point
                                                                         np.dot(eepose,np.append(manip.GetDirection(),0))), # direction
                                                                orpy.IkParameterization.Type.TranslationDirection5D), 
                                        0)
            if soln is not None:
                youbot.SetDOFValues(soln,manip.GetArmIndices())
                yaw_when_j0_is_0 = np.arctan2(direction_when_j0_is_0[1],direction_when_j0_is_0[0]) 
                for pt,yaw in izip(base_pts,base_yaws):
                    basepose = PoseFromXYAndYaw(youbot,pt,yaw)
                    youbot.SetTransform(basepose)
                    joint_angles = soln.copy()
                    joint_angles[0] = yaw_when_j0_is_0-yaw
                    youbot.SetDOFValues(joint_angles,manip.GetArmIndices())
                    if not youbot.CheckSelfCollision():
                        if not checkenvcollision or not youbot.GetEnv().CheckCollision(youbot):
                            if returnfirst:
                                return [basepose],[joint_angles]
                            base_poses.append(basepose)
                            arm_configs.append(joint_angles)
    return base_poses,arm_configs

def VerifyIKSolver(youbot, desired_eepose, visualize=False):
    if visualize:
        h = orpy.misc.DrawAxes(youbot.GetEnv(), desired_eepose, 0.25, 2)
        time.sleep(1)
    base_poses,arm_configs = FindIKSolutions(youbot,desired_eepose,returnfirst=False,checkenvcollision=False)
    print 'Found %d IK solutions. Verifying...'%(len(solns))
    for pose,q in izip(base_poses,arm_configs):
        with youbot:
            youbot.SetTransform(pose)
            youbot.SetDOFValues(q,youbot.GetManipulators()[0].GetArmIndices())
            eepose = youbot.GetManipulators()[0].GetEndEffectorTransform()
            if np.linalg.norm(eepose[:3,3]-desired_eepose[:3,3]) > 0.0001:
                print 'EE is not at the desired position.'
                return
            quat1 = orpy.quatFromRotationMatrix(desired_eepose[:3,:3])
            quat2 = orpy.quatFromRotationMatrix(eepose[:3,:3])
            quatinnerproduct = quat1[0]*quat2[0]+quat1[1]*quat2[1]+quat1[2]*quat2[2]+quat1[3]*quat2[3] 
            quatdist = np.arccos(2.0* (quatinnerproduct*quatinnerproduct) - 1.0) 
            if quatdist > 0.00001:
                print 'EE is not at the desired orientation.'
                return
            if visualize:
                time.sleep(0.3)
    print 'All IK solutions verified.'
    h = None




