#!/usr/bin/env python

## This node directs two robots transporting a wing skin from a start to
## a goal location.  The start location is on a rack, and the goal is
## mounted on the wing ladder.

import roslib; roslib.load_manifest('fleet_control')
from assembly_common.srv import BasePose,ArmCommand
from geometry_msgs.msg import Twist,PoseStamped
from fleet_control.msg import Fleet
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import threading
import rospy
import math
import sys

rospy.init_node('wing_transport')

def arm_command_wrapper(arm_position):
    success = False
    while not success:
        success = True
        try:
            arm_command(arm_position)
        except Exception:
            success = False
            print ("Arm command %s threw an exception. Trying again..."
                         % arm_position);
            rospy.sleep(0.5)


def arm_feedback_wrapper(xx, yy, theta, pos_acc, ang_acc, base_offset, frame):
    print "arm_feedback(%f, %f, %f, %f, %f, %s, %s)" % (xx, yy, theta, pos_acc, ang_acc, base_offset, frame)
    success = False
    while not success:
        success = True
        try:
            arm_feedback(xx, yy, theta, pos_acc, ang_acc, base_offset, frame)
        except Exception:
            success = False
            print "Arm feedback threw an exception. Trying again..."
            rospy.sleep(0.5)


def drive_base(velocity, distance):
    """Use odometry to drive the base."""
    global odom
    cmd = Twist()
    cmd.linear.x = velocity.x
    cmd.linear.y = velocity.y
    cmd.angular.z = velocity.z
    dist2 = distance * distance

    begin = odom
    cmd_vel.publish(cmd)
    while not rospy.is_shutdown():
        current = odom
        dx = current.pose.pose.position.x - begin.pose.pose.position.x
        dy = current.pose.pose.position.y - begin.pose.pose.position.y
        sumsq = dx * dx + dy * dy
        if sumsq >= dist2:
            break

    cmd_vel.publish(Twist())


################################################################


def go_to_rack(args):
    # Call global navigation
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.05
    ang_acc = 0.1
    print "base_command(%f, %f, %f, %f, %f, %s, %s)" % (base_x, base_y, theta, pos_acc, ang_acc, False, "/rack")
    base_command(base_x, base_y, theta, pos_acc, ang_acc, False, "/rack")


def grasp_wing_pose(args):
    # Arm posture command
    arm_command_wrapper('grasp_ladder')


def arm_feedback_to_grasp_wing(args):
    # Local base controller for precise positioning
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.005
    ang_acc = 0.05
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/rack")


def open_gripper(args):
    # Arm posture command
    arm_command_wrapper('gripper_open')


def close_gripper(args):
    # Arm posture command
    arm_command_wrapper('gripper_close')


def back_away(args):
    backward = Vector3()
    (backward.x, backward.y, backward.z) = (-0.04, 0.0, 0.0)
    drive_base(backward, 0.1)
    

###########################################################


cmds = { 'go_to_rack': go_to_rack,
         'grasp_wing_pose': grasp_wing_pose,
         'arm_feedback_to_grasp_wing': arm_feedback_to_grasp_wing,
         'close_gripper': close_gripper,
         'open_gripper': open_gripper,
         'back_away': back_away }


def send_ack(cmd):
    msg = String()
    msg.data = cmd
    print "send ACK: %s" % cmd
    reply_pub.publish(msg)


def distributed_cmd(msg):
    print "Executing distributed command: %s" % msg.data
    split = msg.data.split()
    cmd = split[0]
    cmds[cmd](split)
    send_ack(cmd)


def handle_odom(msg):
    global odom
    odom = msg


# Step 0) Initialization
base_service = 'robot_base_command'
arm_service = 'robot_arm_command'
feedback_service = 'arm_feedback_command'
request_topic = 'wing_transport_request'
reply_topic = 'wing_transport_reply'
cmd_vel_topic = 'cmd_vel'
odom_topic = 'odom'
rospy.wait_for_service(base_service)
rospy.wait_for_service(arm_service)
rospy.wait_for_service(feedback_service)
base_command = rospy.ServiceProxy(base_service, BasePose)
arm_command = rospy.ServiceProxy(arm_service, ArmCommand)
arm_feedback = rospy.ServiceProxy(feedback_service, BasePose)
request_sub = rospy.Subscriber(request_topic, String, distributed_cmd)
reply_pub = rospy.Publisher(reply_topic, String, latch=True)
cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist)
odom_sub = rospy.Subscriber(odom_topic, Odometry, handle_odom)


send_ack('init')

rospy.spin()
