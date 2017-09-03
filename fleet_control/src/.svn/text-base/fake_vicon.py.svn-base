#!/usr/bin/env python

import roslib; roslib.load_manifest('fleet_control')
import rospy
from mit_msgs.msg import MocapPosition,MocapPositionArray
from geometry_msgs.msg import PoseStamped
from euclid import *
import sys
import tf


def quaternion_to_axis_angle(qt):
    eq = Quaternion(qt.w, qt.x, qt.y, qt.z)
    angle, axis = eq.get_angle_axis()
    return axis, angle


def update_poses():
    all_subjects = MocapPositionArray()

    for ii in range(len(names)):
        name = names[ii]
        pub = name_pub[ii]

        subject = MocapPosition()

        ps = PoseStamped()
        ps.header.stamp = rospy.Time(0)
        ps.header.frame_id = '/' + name
        try:
            ps = transformer.transformPose('/map', ps)
        except Exception:
            continue
        (axis, angle) = quaternion_to_axis_angle(ps.pose.orientation)

        subject.name = name
        subject.sample_count = 0
        subject.translational.x = 1000.0 * ps.pose.position.x
        subject.translational.y = 1000.0 * ps.pose.position.y
        subject.translational.z = 1000.0 * ps.pose.position.z
        subject.axisangle.x = angle * axis.x
        subject.axisangle.y = angle * axis.y
        subject.axisangle.z = angle * axis.z

        pub.publish(subject)

        all_subjects.mocap_position.append(subject)

    all_pub.publish(all_subjects)


if __name__ == "__main__":
    rospy.init_node('fake_vicon')
    argv = rospy.myargv(argv=sys.argv)

    if len(argv) < 2:
        print "USAGE: %s list of vicon names" % argv[0]
        exit(1)

    names = []
    name_pub = []
    for arg in argv[1:]:
        names.append(arg)
        name_pub.append(rospy.Publisher('/' + arg, MocapPosition))

    all_pub = rospy.Publisher('/all_subjects', MocapPositionArray)

    transformer = tf.TransformListener()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       update_poses()
       r.sleep()
