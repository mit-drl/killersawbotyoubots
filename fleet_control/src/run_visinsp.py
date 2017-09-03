#!/usr/bin/env python

import roslib; roslib.load_manifest('fleet_control')

import sys
import rospy
from visual_inspection.srv import *
from geometry_msgs.msg import Pose,PoseStamped
import time
import euclid

from vicon_utils import *
import kinect_utils
import numpy as np
import tf

def run_visinsp(pose):
    rospy.wait_for_service("get_visual_inspection")
    try:
        visinsp = rospy.ServiceProxy('get_visual_inspection', GetVisualInspection)
        resp = visinsp(pose)
        return resp
    except rospy.ServiceException, e:   
        print "Service call failed: %s" % e

def usage():
    return "%s [position.x position.y position.z orientation.x orientaton.y orientation.z orientation.w]" % sys.argv[0]

def visual_inspection(objectname,kinectname):
    ps = PoseStamped()
    ps.header.frame_id = objectname
    ps = transform_by_subjects(ps, kinectname)
    return ps.pose # is the pose of the ladder in kinect according to vicon.

def pose_to_mat(pose):
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    mat = np.matrix(tf.transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat


def mat_to_pose(mat, transform = None):
    if transform != None:
        mat = np.dot(transform, mat)
    pose = Pose()
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

if __name__ == "__main__":
    rospy.init_node("run_visinsp")
    time.sleep(3)

    init_pose_mat = kinect_utils.get_subject_in_kinectoptical(kinectvicon_in_vicon=get_subject_transform('kinectdrl'), subject_in_vicon=get_subject_transform('ladder'))
    init_pose_euclid = euclid.Quaternion.new_rotate_matrix(init_pose_mat)
    init_pose = Pose()
    init_pose.position.x = init_pose_mat.d
    init_pose.position.y = init_pose_mat.h
    init_pose.position.z = init_pose_mat.l
    init_pose.orientation.x = init_pose_euclid.x
    init_pose.orientation.y = init_pose_euclid.y
    init_pose.orientation.z = init_pose_euclid.z
    init_pose.orientation.w = init_pose_euclid.w

    delta = Pose()
    delta.position.x = 0.11285; delta.position.y = 0.0867172; delta.position.z -0.0192688
    delta.orientation.x = 0.0191452; delta.orientation.y = -0.0275224; delta.orientation.z = -0.00325887; delta.orientation.w = 0.999433

    init_pose = mat_to_pose(pose_to_mat(init_pose), pose_to_mat(delta))

    print "init_pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)" % \
            (init_pose.position.x, init_pose.position.y, init_pose.position.z, \
             init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z, init_pose.orientation.w)

    resp = run_visinsp(init_pose)

    print "fastner1: %u" % resp.fastner1
    print "fastner2: %u" % resp.fastner2
    print "fastner3: %u" % resp.fastner3
    print "fastner4: %u" % resp.fastner4
