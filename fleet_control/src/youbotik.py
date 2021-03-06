#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import openravepy as orpy
import time
from itertools import izip


PALM_DIRECTION_IN_EE_FRAME = np.array([0.,0.,1.,0.]) # Palm is +z for current youbot openrave robot.
l0 = 0 # FIXME make youbotik a class. make these members.
l1 = 0
l2 = 0
l3 = 0
l4 = 0
j0 = None
j1 = None
j2 = None
j3 = None
j4 = None
j0AnchorInBase = None


def init(youbot):
    global l0,l1,l2,l3,l4
    global j0,j1,j2,j3,j4
    global j0AnchorInBase
    j0 = youbot.GetJoint('j0')
    j1 = youbot.GetJoint('j1')
    j2 = youbot.GetJoint('j2')
    j3 = youbot.GetJoint('j3')
    j4 = youbot.GetJoint('j4')
    # length of links
    l0 = np.linalg.norm(j0.GetAnchor()-j1.GetAnchor())
    l1 = np.linalg.norm(j1.GetAnchor()-j2.GetAnchor())
    l2 = np.linalg.norm(j2.GetAnchor()-j3.GetAnchor())
    l3 = np.linalg.norm(j3.GetAnchor()-j4.GetAnchor())
    l4 = np.linalg.norm(j4.GetAnchor()-youbot.GetManipulators()[0].GetEndEffectorTransform()[:3,3])
    #l1 = 0.15855046593667485 #np.linalg.norm(j1.GetAnchor()-j2.GetAnchor())
    #l2 = 0.13550001368825446 #np.linalg.norm(j2.GetAnchor()-j3.GetAnchor())
    #l3 = 0.029999975673064141 #np.linalg.norm(j3.GetAnchor()-j4.GetAnchor())
    #l4 = 0.14499998766016492 #np.linalg.norm(j4.GetAnchor()-youbot.GetManipulators()[0].GetEndEffectorTransform()[:3,3])

    j0AnchorInBase = np.dot(np.linalg.inv(youbot.GetTransform()),np.append(j0.GetAnchor(),1.))


def AssertUnitVector(vec):
    norm = np.linalg.norm(vec) 
    if norm > 1.00001 or norm < 0.9999:
        raise Exception('Vector not unit')

def IsInLimit(joint,angle):
    if angle >= joint.GetLimits()[0][0] and angle <= joint.GetLimits()[1][0]:
        return True
    return False

def IsSameTransform(tr1, tr2):
    # FIXME just check element-by-element similarity?
    if np.linalg.norm(tr1[:3,3]-tr2[:3,3]) > 0.0001:
        return False
    quat1 = orpy.quatFromRotationMatrix(tr1[:3,:3])
    quat2 = orpy.quatFromRotationMatrix(tr2[:3,:3])
    quatinnerproduct = quat1[0]*quat2[0]+quat1[1]*quat2[1]+quat1[2]*quat2[2]+quat1[3]*quat2[3] 
    quatdist = np.arccos(2.0* (quatinnerproduct*quatinnerproduct) - 1.0) 
    if quatdist > 0.00001:
        return False
    return True
 
def NormalizeAngle(angle):
    while angle < -np.pi:
        angle += 2.0*np.pi
    while angle > np.pi:
        angle -= 2.0*np.pi
    return angle

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
    if isinstance(extent,tuple): # FIXME Don't exactly know why I have to do this, but extent sometimes is in a tuple..?
        extent = extent[0]
    if extent > np.pi:
        print 'Warning: rotation extent %f is truncated to pi.'%(extent)
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

# position is in 2d (x,y). Use current z of robot.
# direction is in 2d (x,y).
def GetBasePosesGivenArmLink0Pose(youbot,armlink0position,armlink0direction,j0resolution):
    current_basepose = youbot.GetLinks()[0].GetTransform()
    current_armlink0pose = youbot.GetLinks()[1].GetTransform()
    base_to_armlink0_planar_dist = np.linalg.norm(current_basepose[:2,3]-current_armlink0pose[:2,3])
    extent=np.min((abs(youbot.GetJoints()[0].GetLimits()[0][0]),abs(youbot.GetJoints()[0].GetLimits()[1][0]))), # Assuming almost symmetric min and max limits for j0.
    #print 'for j0. extent: %f'%(extent)
    base_pts,neg_base_angles = DiscretizeCircleAroundDirection(center=armlink0position,
                                                               direction=-1.0*armlink0direction,
                                                               radius=base_to_armlink0_planar_dist,
                                                               extent=extent,
                                                               resolution=j0resolution)
    base_angles = [angle+np.pi for angle in neg_base_angles]
    return base_pts,base_angles


def SolveJ1J2(youbot,j1anchor,j0heading,j3anchor):
    AssertUnitVector(j0heading)
    # this function uses j1,j2,j3 triangle.
    j1Toj3Vector = j3anchor - j1anchor 
    h = np.linalg.norm(j1Toj3Vector)
    if h >= l1+l2 or h <= abs(l1-l2):
        return []
    l1sqr = l1*l1
    l2sqr = l2*l2
    hsqr = h*h
    th1 = np.arccos((l1sqr+hsqr-l2sqr)/(2.*l1*h)) # inverse of cosine formula
    th2 = np.arccos((l1sqr+l2sqr-hsqr)/(2.*l1*l2)) # inverse of cosine formula
    z = np.array([0.,0.,1.])
    phi1 = np.arccos(np.dot(j1Toj3Vector,z)/h) # angle between j1ToJ3vector and +z (unit vector) in world.
    if np.dot(j0heading,j1Toj3Vector) < 0:
        phi1 = -phi1
    results = [] 
    # righty
    j1angle = NormalizeAngle(phi1 + th1)
    j2angle = NormalizeAngle(-(np.pi-th2))
    if IsInLimit(j1,j1angle) and IsInLimit(j2,j2angle):
        results.append((j1angle,j2angle))
    # lefty
    j1angle = NormalizeAngle(phi1 - th1)
    j2angle = NormalizeAngle(np.pi - th2)
    if IsInLimit(j1,j1angle) and IsInLimit(j2,j2angle):
        results.append((j1angle,j2angle))
    return results

def SolveJ1J2J3(youbot,j1anchor,j0heading,eeposition,palmdirection):
    AssertUnitVector(palmdirection)
    AssertUnitVector(j0heading)
    j3anchor = (l3+l4)*(-1.0*palmdirection)+eeposition
    j1j2solns = SolveJ1J2(youbot,j1anchor,j0heading,j3anchor)
    z = np.array([0.,0.,1.])
    phi3 = np.arccos(np.dot(palmdirection,z)) # angle between palmdirection and +z (both unit vectors) in world.
    if np.dot(j0heading,palmdirection) < 0:
        phi3 = -phi3
    results = []
    for (j1angle,j2angle) in j1j2solns:
        j3angle = NormalizeAngle(phi3-(j1angle+j2angle))
        if IsInLimit(j3,j3angle):
            results.append((j1angle,j2angle,j3angle))
    return results

def SolveJ4(youbot,eepose,j0j1j2j3):
    with youbot:
        youbot.SetDOFValues(np.append(j0j1j2j3,0.),[0,1,2,3,4])
        link4tr = youbot.GetLink('link4').GetTransform()
        eerot_in_link4 = np.dot(np.linalg.inv(link4tr[:3,:3]),eepose[:3,:3])
        j4angle = np.arctan2(eerot_in_link4[1,0],eerot_in_link4[0,0])
    if IsInLimit(j4,j4angle):
        return j4angle
    else:
        return None

def ComputeJ1Anchor(youbot,j0heading,j0anchor):
    AssertUnitVector(j0heading) 
    j0Toj1Vector = j1.GetAnchor() - j0.GetAnchor()
    lengthInXY = np.linalg.norm(j0Toj1Vector[:2])
    j1anchor = np.array([0.,0.,0.])
    j1anchor[:2] = j0anchor[:2]+(j0heading*lengthInXY)[:2]
    j1anchor[2] = j0anchor[2]+j0Toj1Vector[2] # use the current z as height
    return j1anchor

def ComputeJ0Anchor(youbot,basepose):
    hom = np.dot(basepose,j0AnchorInBase)
    return hom[:3]

def SolveJ0J1J2J3(youbot,j0anchor,j0heading,basepose,eepose,palmdirection):
    AssertUnitVector(j0heading)
    baseyaw = np.arctan2(basepose[1,0],basepose[0,0])
    j0headingyaw = np.arctan2(j0heading[1],j0heading[0])
    j0angle = NormalizeAngle(-1.0*(j0headingyaw-baseyaw)) # j0 axis looks down, not up, hence the -1.0.
    results = []
    if IsInLimit(j0,j0angle):
        j1anchor = ComputeJ1Anchor(youbot,j0heading,j0anchor)
        j1j2j3solns = SolveJ1J2J3(youbot,j1anchor,j0heading,eepose[:3,3],palmdirection)
        for (j1angle,j2angle,j3angle) in j1j2j3solns:
            results.append((j0angle,j1angle,j2angle,j3angle))
    return results

def FindIKSolutions(youbot,eepose,checkenvcollision=True,rotationresolution=0.05):
    basepose = youbot.GetTransform()
    j0anchor = youbot.GetJoint('j0').GetAnchor()
    palmdirection = np.dot(eepose,PALM_DIRECTION_IN_EE_FRAME)[:3]
    j0ToEE = (eepose[:3,3]-j0anchor)[:2]
    norm = np.linalg.norm(j0ToEE)
    palmdirnorm = np.linalg.norm(palmdirection[:2])
    solns = []
    j0headings = []
    if norm < 0.00001 and palmdirnorm < 0.00001:
        # End-effector is exactly over j0 and palm is looking straight up/down. 
        # Any j0heading within limits should work.
        extent=np.min((abs(youbot.GetJoints()[0].GetLimits()[0][0]),abs(youbot.GetJoints()[0].GetLimits()[1][0]))), # Assuming almost symmetric min and max limits for j0.
        for angle in DiscretizeAnglesAroundDirection(youbot.GetTransform()[:2,0],extent,rotationresolution):
            j0headings.append(np.array([np.cos(angle),np.sin(angle),0.]))  
    else:
        # Only two j0headings can work.
        if norm >= 0.00001:
            direction = np.append(j0ToEE/norm,0)
        else:
            direction = np.append(palmdirection[:2]/palmdirnorm,0.)
        j0headings.append(direction)
        j0headings.append(-1.0*direction)
    j0j1j2j3solns = []
    for j0heading in j0headings:
        j0j1j2j3solns.extend( SolveJ0J1J2J3(youbot,j0anchor,j0heading,basepose,eepose,palmdirection) )
    if j0j1j2j3solns is not None and len(j0j1j2j3solns) > 0:
        for (j0angle,j1angle,j2angle,j3angle) in j0j1j2j3solns:
            j4angle = SolveJ4(youbot,eepose,np.array([j0angle,j1angle,j2angle,j3angle])) 
            if j4angle is not None:
                solns.append(np.array([j0angle,j1angle,j2angle,j3angle,j4angle]))
    noncollidingsolns = []
    for soln in solns:
        youbot.SetDOFValues(soln,range(5))
        if youbot.CheckSelfCollision():
            continue
        if checkenvcollision and youbot.GetEnv().CheckCollision(youbot):
            continue
        noncollidingsolns.append(soln)
    return noncollidingsolns

def FindIKAndBaseSolutions(youbot, eepose, 
                           returnfirst=False, checkenvcollision=True, 
                           translationextent=0.60, translationresolution=0.01, rotationextent=np.pi, rotationresolution=0.05):
                           
    #youbot = openrave robot
    #eepose = 4x4 numpy array
    
                           
    manip = youbot.GetManipulators()[0]
    palmdirection = np.dot(eepose,PALM_DIRECTION_IN_EE_FRAME)[:2]
    norm = np.linalg.norm(palmdirection)
    arm_link0_positions = []
    base_directions_with_j0_set_to_0 = []
    if norm == 0:
        # This means the palm is looking up or down. A singular case where all search directions will work. Use current robot direction as main direction.
        for angle in DiscretizeAnglesAroundDirection(youbot.GetTransform()[:2,0],extent,rotationresolution):
            axis = np.array([np.cos(angle), np.sin(angle)])
            newpositions = DiscretizeLineAroundPoint(eepose[:2,3],axis,extent=translationextent,resolution=translationresolution)
            arm_link0_positions.extend(newpositions)
            base_directions_with_j0_set_to_0.extend([axis]*len(newpositions))
    else:
        arm_link0_positions = []
        positions = DiscretizeLineAroundPoint(eepose[:2,3],palmdirection/norm,extent=translationextent,resolution=translationresolution)
        base_directions_with_j0_set_to_0 = [palmdirection/norm]*len(positions)
        arm_link0_positions.extend(positions)
        positions_180 = positions
        base_directions_with_j0_set_to_0.extend([-1.0*palmdirection/norm]*len(positions_180))
        arm_link0_positions.extend(positions_180)
    base_poses = []
    arm_configs = []
    for position,direction_when_j0_is_0 in izip(arm_link0_positions,base_directions_with_j0_set_to_0):
        base_pts,base_yaws = GetBasePosesGivenArmLink0Pose(youbot,position,direction_when_j0_is_0,j0resolution=rotationresolution)
        with youbot:
            youbot.SetTransform(PoseFromXYAndYaw(youbot,base_pts[0],base_yaws[0]))
            solns = FindIKSolutions(youbot,eepose,checkenvcollision=False,rotationresolution=rotationresolution)
            if len(solns) > 0:
                #The solution is either for the correct ee-pose or for a 180 rotation around the direction given to the ik solver.
                for soln in solns:
                    youbot.SetDOFValues(soln,manip.GetArmIndices())
                    yaw_when_j0_is_0 = np.arctan2(direction_when_j0_is_0[1],direction_when_j0_is_0[0]) 
                    for pt,yaw in izip(base_pts,base_yaws):
                        basepose = PoseFromXYAndYaw(youbot,pt,yaw)
                        youbot.SetTransform(basepose)
                        joint_angles = soln.copy()
                        joint_angles[0] = -1.0 * NormalizeAngle(yaw_when_j0_is_0-yaw) # j0 rotation axis looks down, hence the -1.0.
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
    base_poses,arm_configs = FindIKAndBaseSolutions(youbot,desired_eepose,returnfirst=False,checkenvcollision=False)
    print 'Found %d IK solutions. Verifying...'%(len(base_poses))
    youbot.GetController().Reset()
    for pose,q,i in izip(base_poses,arm_configs,range(len(base_poses))):
        with youbot:
            youbot.SetTransform(pose)
            youbot.SetDOFValues(q,youbot.GetManipulators()[0].GetArmIndices())
            eepose = youbot.GetManipulators()[0].GetEndEffectorTransform()
            if np.linalg.norm(eepose[:3,3]-desired_eepose[:3,3]) > 0.005:
                print 'EE is not at the desired position. index: %d.'%(i)
                return base_poses,arm_configs
            quat1 = orpy.quatFromRotationMatrix(desired_eepose[:3,:3])
            quat2 = orpy.quatFromRotationMatrix(eepose[:3,:3])
            quatinnerproduct = quat1[0]*quat2[0]+quat1[1]*quat2[1]+quat1[2]*quat2[2]+quat1[3]*quat2[3] 
            quatdist = np.arccos(2.0* (quatinnerproduct*quatinnerproduct) - 1.0) 
            if quatdist > 0.001:
                print 'EE is not at the desired orientation. index: %d.'%(i)
                return base_poses,arm_configs
            if visualize:
                youbot.GetEnv().UpdatePublishedBodies()
                time.sleep(0.001)
    print 'All IK solutions verified.'
    h = None
    return base_poses,arm_configs

