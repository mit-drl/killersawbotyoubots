#!/usr/bin/env python

## This node directs two robots transporting a wing skin from a start to
## a goal location.  The start location is on a rack, and the goal is
## mounted on the wing ladder.

import roslib; roslib.load_manifest('fleet_control')
from assembly_common.srv import BasePose,ArmCommand
from assembly_common.msg import CommunicationStamped, Communication
from geometry_msgs.msg import Twist,PoseStamped,Point
from fleet_control.msg import Fleet
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Header
import threading
import rospy
import math
import sys
import time
import euclid
from hole_detection.msg import Hole
from brics_actuator.msg import JointPositions, JointValue
from mit_msgs.msg import MocapPosition
import numpy as np

rospy.init_node('wood_transport')

#my_position = None

class myvar(object):
    def __init__(self,name,value):
        self.name = name
        self.value = value
        print self.name, ' init to ',self.value
    def set_value(self,value):
        self.value=value
        print self.name, ' set to ',self.value
    def read_value(self):
        print self.name, ' read with value ',self.value
        return self.value

last_cmd_success = True



def drive_base(velocity, distance):
    """Use odometry to drive the base."""
    global odom
    cmd = Twist()
    cmd.linear.x = velocity.x
    cmd.linear.y = velocity.y
    cmd.angular.z = velocity.z
    dist2 = distance * distance

    begin = odom
    cmd_vel_pub.publish(cmd)
    while not rospy.is_shutdown():
        current = odom
        dx = current.pose.pose.position.x - begin.pose.pose.position.x
        dy = current.pose.pose.position.y - begin.pose.pose.position.y
        sumsq = dx * dx + dy * dy
        if sumsq >= dist2:
            break

    cmd_vel_pub.publish(Twist())


################################################################


def go_to_wood(args):
    # Call global navigation
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.01
    ang_acc = 0.1
    print "base_command(%f, %f, %f, %f, %f, %s, %s)" % (base_x, base_y, theta, pos_acc, ang_acc, False, "/wood0")
    base_command(base_x, base_y, theta, pos_acc, ang_acc, False, "/wood0")










def halt(args):
    #TODO: will this work?
    arm_command_wrapper('gripper_close')
    drive_base(Vector3(), 0.0)





###########################################################

print "Creating cmds"

#TODO: these sorts of things should really just use reflection
cmds = { 'go_to_rack': go_to_wood,
         'halt' : halt}

print "Finished Cmds"


def send_ack(cmd):
    msg = String()
    msg.data = cmd
    print "send ACK: %s" % cmd
    reply_pub.publish(msg)


def distributed_cmd(msg):
    print "Executing distributed command: %s" % msg.data
    split = msg.data.split()
    cmd = split[0]
   # communication_pub.publish(CommunicationStamped(
   #                     header=Header(frame_id='map'),
   #                     comm=Communication(position=Point(x=my_position[0], y=my_position[1], z=my_position[2]))
   #                     ))
    cmds[cmd](split)
    send_ack(cmd)


def handle_odom(msg):
    global odom
    odom = msg


def DidLastCommandSucceed(msg):
    global last_cmd_success
    print 'did last cmd succed returning: ',last_cmd_success
    return CheckSuccessResponse(success=last_cmd_success)


my_name = rospy.get_namespace()
# Step 0) Initialization
print "initialize"
base_service = 'robot_base_command'
arm_service = 'robot_arm_command'
feedback_service = 'arm_feedback_command'
request_topic = 'wood_transport_request'
reply_topic = 'wood_transport_reply'
arm_pos_topic = my_name + 'arm_1/arm_controller/position_command'
cmd_vel_topic = 'cmd_vel'
communication_topic = my_name + 'communication'
odom_topic = 'odom'
position_topic = my_name
print my_name+ ': waiting for base service'
rospy.wait_for_service(base_service)
print my_name+ ':waiting for arm service'
rospy.wait_for_service(arm_service)
print my_name+ ': waiting for feedback service'
rospy.wait_for_service(feedback_service)
print my_name+ ':done waiting for services'
base_command = rospy.ServiceProxy(base_service, BasePose)
arm_command = rospy.ServiceProxy(arm_service, ArmCommand)
arm_feedback = rospy.ServiceProxy(feedback_service, BasePose)
request_sub = rospy.Subscriber(request_topic, String, distributed_cmd)
reply_pub = rospy.Publisher(reply_topic, String, latch=True)
cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist)
odom_sub = rospy.Subscriber(odom_topic, Odometry, handle_odom)
arm_pos_pub = rospy.Publisher(arm_pos_topic, JointPositions, latch=True)
#position_sub = rospy.Subscriber(position_topic, MocapPosition, set_pos)
communication_pub = rospy.Publisher('/communication', CommunicationStamped, latch=True)

s = rospy.Service('did_last_command_succeed', CheckSuccess, DidLastCommandSucceed)

rospy.sleep(1.0)

send_ack('init')


rospy.spin()
