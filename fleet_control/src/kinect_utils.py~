## Routines to help with kinect

import roslib
from geometry_msgs.msg import PoseStamped,Pose,PointStamped,Vector3Stamped,Point
from mit_msgs.msg import ObjectPose
from euclid import *
import rospy
import math
import numpy as np
import sys
import tf
import tf.transformations as tr
import time


all_subjects = {}
watch_list = []
origin = None
want_inv = None
my_name = None
my_frame = None

def get_subject_in_kinectoptical(kinectvicon_in_vicon, subject_in_vicon):
    #takes in a transform listener and an offset and a target name "/<target>" - 
    #the offset is the offset from the ladder at which we expect the target object.
    #while True:
    #    try:
    #        (trans,rot) = listener.lookupTransform('/kinect', target, rospy.Time(0))
    #        break
    #    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #        print "TF buffering problem...retrying..."
    #        rospy.sleep(1.0)
	#	euler = tr.euler_from_quaternion(rot)
    #return np.add(tr.compose_matrix(angles=euler, translate=trans), offset)
    kinectvicon_in_kinectoptical = Matrix4()
    kinectvicon_in_kinectoptical.a = -0.99739574107415485
    kinectvicon_in_kinectoptical.b = -0.055204502142269039
    kinectvicon_in_kinectoptical.c = -0.046413345390763618
    kinectvicon_in_kinectoptical.d = -0.080685522157861816
    kinectvicon_in_kinectoptical.e = 0.023297061990552016
    kinectvicon_in_kinectoptical.f = 0.36243100569095482
    kinectvicon_in_kinectoptical.g = -0.9317193853389828
    kinectvicon_in_kinectoptical.h = -0.054623377896707105
    kinectvicon_in_kinectoptical.i = 0.068256740251395581
    kinectvicon_in_kinectoptical.j = -0.93037424139808778
    kinectvicon_in_kinectoptical.k = -0.36020103880081511
    kinectvicon_in_kinectoptical.l = 0.059874154231637278
    kinectvicon_in_kinectoptical.m = 0.0
    kinectvicon_in_kinectoptical.n = 0.0
    kinectvicon_in_kinectoptical.o = 0.0
    kinectvicon_in_kinectoptical.p = 1.0000000000000002

    vicon_in_kinectvicon = kinectvicon_in_vicon.inverse()

    subject_in_kinectoptical = kinectvicon_in_kinectoptical * vicon_in_kinectvicon * subject_in_vicon
    return subject_in_kinectoptical
	


def get_subject(subject):
    while not rospy.is_shutdown():
        try:
            mp = all_subjects[subject]
            if mp.origin.x != 0.0 or mp.origin.y != 0.0 or mp.origin.z != 0.0:
                return mp
        except KeyError:
            #print 'subject: %s'%(subject)
            #print 'all keys: ',list(all_subjects.keys())
            pass
        rospy.sleep(0.01)

    sys.exit(0) # Don't return None; if ROS is finished, just help it along ...


def axisangle_to_quaternion(aa):
    axis = Vector3()
    axis.x = aa.x
    axis.y = aa.y
    axis.z = aa.z
    angle = math.sqrt(aa.x * aa.x + aa.y * aa.y + aa.z * aa.z)
    return Quaternion.new_rotate_axis(angle, axis)


def rpy_to_matrix(rpy):
    m = Matrix4()
    ca = np.cos(rpy.z)
    sa = np.sin(rpy.z)
    cb = np.cos(rpy.y)
    sb = np.sin(rpy.y)
    cg = np.cos(rpy.x)
    sg = np.sin(rpy.x)
    m[0] = ca*cb
    m[1] = ca*sb*sg-sa*cg
    m[2] = ca*sb*cg+sa*sg
    m[4] = sa*cb 
    m[5] = sa*sb*sg+ca*cg
    m[6] = sa*sb*cg-ca*sg
    m[8] = -sb
    m[9] = cb*sg
    m[10] = cb*cg
    return m

def rpy_to_matrix_correct(rpy):
    m = Matrix4()
    ca = np.cos(rpy.z)
    sa = np.sin(rpy.z)
    cb = np.cos(rpy.y)
    sb = np.sin(rpy.y)
    cg = np.cos(rpy.x)
    sg = np.sin(rpy.x)
    m.a = ca*cb
    m.b = ca*sb*sg-sa*cg
    m.c = ca*sb*cg+sa*sg
    m.e = sa*cb 
    m.f = sa*sb*sg+ca*cg
    m.g = sa*sb*cg-ca*sg
    m.i = -sb
    m.j = cb*sg
    m.k = cb*cg
    return m


def get_subject_pose(frame):
    while frame[0] == '/':
        frame = frame[1:]
    part = get_subject(frame)
    pt = Point3()
    pt.x = part.origin.x
    pt.y = part.origin.y
    pt.z = part.origin.z
    rot = rpy_to_matrix(part.rpy)
    pose = Pose3()
    return pose.new_pose(pt, rot)

def get_subject_matrix(frame):
    while frame[0] == '/':
        frame = frame[1:]
    part = get_subject(frame)
    mat = rpy_to_matrix(part.rpy)
    mat[3] = part.origin.x
    mat[7] = part.origin.y
    mat[11] = part.origin.z
    return mat

def get_subject_matrix_correct(frame):
    while frame[0] == '/':
        frame = frame[1:]
    part = get_subject(frame)
    mat = rpy_to_matrix_correct(part.rpy)
    mat.d = part.origin.x
    mat.h = part.origin.y
    mat.l = part.origin.z
    return mat

def subject_name_ok(name):
    # NOTE: this could fail transliently
    return 'map' == name or name in all_subjects

def handle_subjects(msg):
    global all_subjects
    #if msg.origin.x == 0.0 and msg.origin.y == 0.0 and msg.origin.z == 0.0:
    #    # FOR now ignore these. kinect seems to be misbehaving.
    #    return
    all_subjects[msg.name] = msg
    #for part in msg.mocap_position:
    #    if part.translational.x == 0.0 and part.translational.y == 0.0 and part.translational.z == 0.0 and part.axisangle.x == 0.0 and part.axisangle.y == 0.0 and part.axisangle.z == 0.0:
    #        if part.name in all_subjects:
    #            del all_subjects[part.name]
    #    else:
    #        all_subjects[part.name] = part

vicon_sub = rospy.Subscriber('/output', ObjectPose,
                             handle_subjects)


