#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('fleet_control')
import rospy
from mit_msgs.msg import MocapPositionArray
from visual_inspection.srv import *
from geometry_msgs.msg import Pose
from tf.transformations import * # for quat <-> Euler <-> rotation matrix
import numpy as np
from vicon_utils import *
import kinect_utils
import euclid
import tf

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


def run_visinsp_vicon(target):
    # get pose of 'ladder' in kinect coordinate frame
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

    # adjust calibration error
    delta = Pose()
    delta.position.x = 0.11285; delta.position.y = 0.0867172; delta.position.z -0.0192688
    delta.orientation.x = 0.0191452; delta.orientation.y = -0.0275224; delta.orientation.z = -0.00325887; delta.orientation.w = 0.999433

    init_pose = mat_to_pose(pose_to_mat(init_pose), pose_to_mat(delta))

    print "init_pose: position(%f, %f, %f), orientation(%f, %f, %f, %f)" % \
            (init_pose.position.x, init_pose.position.y, init_pose.position.z, \
             init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z, init_pose.orientation.w)

    rospy.wait_for_service("get_visual_inspection")
    try:
        visinsp = rospy.ServiceProxy('get_visual_inspection', GetVisualInspection)
        resp = visinsp(init_pose)
        print "fastner1: %u" % resp.fastner1
        print "fastner2: %u" % resp.fastner2
        print "fastner3: %u" % resp.fastner3
        print "fastner4: %u" % resp.fastner4

        if target == 'fastner1':
            return resp.fastner1
        elif target == 'fastner2':
            return resp.fastner2
        elif target == 'fastner3':
            return resp.fastner3
        elif target == 'fastner4':
            return resp.fastner4
        else: # unkown target name, return False
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def get_quaternion(alpha, beta, gamma):
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = rotation_matrix(alpha, xaxis)
    Ry = rotation_matrix(beta, yaxis)
    Rz = rotation_matrix(gamma, zaxis)
    R = concatenate_matrices(Rz, Ry, Rx)
    q = quaternion_from_matrix(R)
    return q

def usage():
    return "%s" % sys.argv[0]

if __name__ == "__main__":
    init_pose = Pose()
    if len(sys.argv) == 1:
        pass
    else:
        print usage()
        sys.exit(1)

    rospy.set_param('visual_inspection/cloud_buf_size', 30)
    rospy.set_param('visual_inspection/numof_skip_dummy_clouds', 30)

    rospy.set_param('visual_inspection/th_nn', 5)
    rospy.set_param('/visual_inspection/vg_leaf_coarse', 0.01)
    rospy.set_param('/visual_inspection/icp_max_iter', 10)


    rospy.init_node("run_visinsp")

    resp = run_visinsp_vicon('fastner1')

    # print "fastner1: %u" % resp

