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
from fleet_control.srv import SearchHole,SearchHoleResponse,InsertToHole,CheckSuccess,CheckSuccessResponse
from hole_detection.msg import Hole
from brics_actuator.msg import JointPositions, JointValue
from mit_msgs.msg import MocapPosition
import numpy as np

rospy.init_node('wing_transport')

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

#def set_pos(msg):
#    global my_position
#    my_position = np.array([msg.translational.x / 1000.0,
#                            msg.translational.y / 1000.0,
#                            msg.translational.z / 1000.0])

def arm_command_wrapper(arm_position):
    success = False
    while not success:
        btime = time.time()
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


def go_to_rack(args):
    # Call global navigation
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.01
    ang_acc = 0.1
    print "base_command(%f, %f, %f, %f, %f, %s, %s)" % (base_x, base_y, theta, pos_acc, ang_acc, False, "/youbot_triangle")
    base_command(base_x, base_y, theta, pos_acc, ang_acc, False, "/youbot_triangle")

def go_to_ladder(args):
    # Call global navigation
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.05
    ang_acc = 0.1
    print "base_command(%f, %f, %f, %f, %f, %s, %s)" % (base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")
    base_command(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def go_to_drc4(args):
    # Call global navigation
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.05
    ang_acc = 0.1
    print "base_command(%f, %f, %f, %f, %f, %s, %s)" % (base_x, base_y, theta, pos_acc, ang_acc, False, "/drc4")
    base_command(base_x, base_y, theta, pos_acc, ang_acc, False, "/drc4")


def grasp_wing_pose(args):
    # Arm posture command
    arm_command_wrapper('grasp_ladder')

def grasp_wing_side_pose(args):
    # Arm posture command
    for i in range(3):
        arm_command_wrapper('grasp_triangle')
        rospy.sleep(0.1)

def push_pose(args):
    arm_command_wrapper('push_skin_pose')

def arm_up_pose(args):
    arm_command_wrapper('arm_carry')

def slide_pose(args):
    arm_command_wrapper('slide_pose')

def arm_feedback_to_grasp_wing(args):
    # Local base controller for precise positioning
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.005
    ang_acc = 0.05
    #TODO: add necessary offsets to base_x and base_y
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/youbot_triangle")

def arm_feedback_to_grasp_wing_side(args):
    # Local base controller for precise positioning
    base_x = float(args[1])
    base_y = float(args[2])
    theta = float(args[3])
    pos_acc = 0.005
    ang_acc = 0.05
    #TODO: add necessary offsets to base_x and base_y
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/youbot_triangle")

def open_gripper(args):
    # Arm posture command
    arm_command_wrapper('gripper_open')
    rospy.sleep(0.1)

def compliant_grip(args):
    # Arm posture command
    arm_command_wrapper('compliant_grip')
    rospy.sleep(0.1)

def close_gripper(args):
    # Arm posture command
    arm_command_wrapper('gripper_close')
    rospy.sleep(0.1)

def rotate_skin(args):
    if args[0] == 'neg':
        arm_command_wrapper('rotate_hand_neg')
    else:
        arm_command_wrapper('rotate_hand_pos')

def align_with_hole(args):
    # Local base controller for precise positioning
    #base_x = 0.322
    #base_y = 0.148
    #theta = -math.pi/2.0 + 0.2
    base_x = 0.314 #0.322
    base_y = 0.188 #0.185
    theta = -1.582 #-1.576
    pos_acc = 0.002
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def align_with_hole_2(args):
    # Local base controller for precise positioning
    base_x = -0.294 #-0.291
    base_y = 0.168 #0.171
    theta = -1.543 #-1.569
    pos_acc = 0.002
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def align_with_hole_3(args):
    # Local base controller for precise positioning
    base_x = -0.328 + 0.01 #-0.322 #-0.331, -0.156
    base_y = -0.169 #-0.156
    theta = 1.567 #1.563
    pos_acc = 0.002
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def align_with_hole_4(args):
    # Local base controller for precise positioning
    base_x = 0.286 + 0.01 #0.288 + 0.013
    base_y = -0.170 #-0.175
    theta = 1.551 #1.511
    pos_acc = 0.002
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def push_first(args):
    # Local base controller for precise positioning
    base_x = -0.320
    base_y = -0.223#-0.218
    theta = math.pi/2.0
    pos_acc = 0.001
    ang_acc = 0.01
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")
    #base_y -= 0.200
    #arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def push_first_backup(args):
    # Local base controller for precise positioning
    base_x = -0.320
    base_y = -0.5#-0.218
    theta = math.pi/2.0
    pos_acc = 0.001
    ang_acc = 0.01
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def push_second(args):
    # Local base controller for precise positioning
    base_x = -0.325
    base_y = 0.178
    theta = -math.pi/2.0
    pos_acc = 0.001
    ang_acc = 0.01
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")
    #base_y += 0.200
    #arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def push_second_backup(args):
    # Local base controller for precise positioning
    base_x = -0.325
    base_y = 0.5
    theta = -math.pi/2.0
    pos_acc = 0.001
    ang_acc = 0.01
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def hole1_inspection_pose(args):
    base_x = 0.767
    base_y = 0.522
    theta =  -2.554
    pos_acc = 0.003
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def hole2_inspection_pose(args):
    base_x = -0.814
    base_y = 0.562
    theta =  -0.372
    pos_acc = 0.003
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def hole3_inspection_pose(args):
    base_x = -0.743
    base_y = -0.04
    theta = 0.106
    pos_acc = 0.003
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def hole4_inspection_pose(args):
    base_x = 0.835
    base_y = 0.041
    theta = -3.048
    pos_acc = 0.003
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")

def back_away(args):
    backward = euclid.Vector3()
    (backward.x, backward.y, backward.z) = (-0.15, 0.0, 0.0)
    drive_base(backward, 0.22)

def back_away_right(args):
    backward = euclid.Vector3()
    (backward.x, backward.y, backward.z) = (-0.05, -0.15, 0.0)
    drive_base(backward, 0.35)

def circular_motion(args):
    base_x = -0.332
    #base_y = -0.022
    base_y = 0.03
    theta = 0.093
    pos_acc = 0.003
    ang_acc = 0.02
    arm_feedback_wrapper(base_x, base_y, theta, pos_acc, ang_acc, False, "/ladder")
    #backward = euclid.Vector3()
    #(backward.x, backward.y, backward.z) = (0.0, 0.01, 0.0)
    #drive_base(backward, 0.085)


def back_away_slightly(args):
    backward = euclid.Vector3()
    (backward.x, backward.y, backward.z) = (-0.07, 0.0, 0.0)
    drive_base(backward, 0.005)


def halt(args):
    #TODO: will this work?
    arm_command_wrapper('gripper_close')
    drive_base(Vector3(), 0.0)

def set_joint_5(args):
    beginning_val = 2.90
    angle = float(args[1])
    jp = JointPositions()
    jp.positions.append(JointValue())
    jp.positions[0].joint_uri = 'arm_joint_5'
    jp.positions[0].unit = 'rad'
    #TODO: fix this hack

    if my_name == '/drc1/':
        jp.positions[0].value = beginning_val - angle
    else:
        jp.positions[0].value = beginning_val + angle
    print jp
    rospy.sleep(0.1)
    for i in range(1):
        arm_pos_pub.publish(jp)
        rospy.sleep(0.1)

def arm_inspection_config_tilted(args):
    jp = JointPositions()
    jp.positions.append(JointValue())
    jp.positions[0].joint_uri = 'arm_joint_5'
    jp.positions[0].unit = 'rad'
    jp.positions[0].value = 3.2234
    rospy.sleep(0.1)
    arm_pos_pub.publish(jp)
    rospy.sleep(0.1)

def arm_inspection_config(args):
    jp = JointPositions()
    jp.positions.append(JointValue())
    jp.positions[0].joint_uri = 'arm_joint_5'
    jp.positions[0].unit = 'rad'
    jp.positions[0].value = 2.9234
    rospy.sleep(0.1)
    arm_pos_pub.publish(jp)
    rospy.sleep(0.1)

def Insert(args):
    print args
    angle = Float64(data=float(args[1]))
    global hole
    if hole is None:
        hole = Hole(found=False,width=-1.0,position=Point(x=0.0,y=0.0,z=0.0))
    success = insert_service(hole, angle,args[2])
    print "Did we succed?: ", success
    #TODO: what to do on failures?  Should this actually be a service call of some sort?

def LiftTool(args):
    global hole
    success = lifttool_service(hole, Float64(data=0.0),args[2])

#def initialize(args):
#s = String()
#s.data = 'start'
#initialize_publisher.Publish(s)

#TODO: generalize all of this

def search_ladder(args):
    global hole
    hole = Hole()
    hole.found = False
    firsttime = True
    while not hole.found and not rospy.is_shutdown():
        if not firsttime:
            align_with_hole(None)
        firsttime = False
        resp = search_service('ladder_hole')
        hole = resp.hole
        while hole.found and abs(hole.position.x) > 0.0015 and not rospy.is_shutdown():
            resp = search_service('ladder_hole')
            hole = resp.hole

def search_ladder_2(args):
    global hole
    hole = Hole()
    hole.found = False
    firsttime = True
    while not hole.found and not rospy.is_shutdown():
        if not firsttime:
            align_with_hole_2(None)
        firsttime = False
        resp = search_service('ladder_hole_2')
        hole = resp.hole
        while hole.found and abs(hole.position.x) > 0.003 and not rospy.is_shutdown():
            resp = search_service('ladder_hole_2')
            hole = resp.hole

def search_ladder_2_bottom(args):
    global hole
    hole = Hole()
    hole.found = False
    firsttime = True
    while not hole.found and not rospy.is_shutdown():
        if not firsttime:
            align_with_hole_2(None)
        firsttime = False
        resp = search_service('ladder_hole_2_bottom')
        hole = resp.hole
        while hole.found and abs(hole.position.x) > 0.003 and not rospy.is_shutdown():
            resp = search_service('ladder_hole_2_bottom')
            hole = resp.hole

def search_ladder_fine(args):
    global hole
    resp = search_service('ladder_hole_fine')
    hole = resp.hole
    while not hole.found and not rospy.is_shutdown():
        search_ladder(None)
        resp = search_service('ladder_hole_fine')
        hole = resp.hole

def search_ladder_fine_2(args):
    global hole
    resp = search_service('ladder_hole_fine_2')
    hole = resp.hole
    while not hole.found and not rospy.is_shutdown():
        search_ladder_2(None)
        resp = search_service('ladder_hole_fine_2')
        hole = resp.hole

def search_ladder_fine_2_bottom(args):
    global hole
    resp = search_service('ladder_hole_fine_2_bottom')
    hole = resp.hole
    while not hole.found and not rospy.is_shutdown():
        search_ladder_2(None)
        resp = search_service('ladder_hole_fine_2_bottom')
        hole = resp.hole


def search_ladder_3(args):
    global hole
    hole = Hole()
    hole.found = False
    firsttime = True
    while not hole.found and not rospy.is_shutdown():
        if not firsttime:
            align_with_hole_3(None)
        firsttime = False
        resp = search_service('ladder_hole_3')
        hole = resp.hole
        while hole.found and abs(hole.position.x) > 0.003 and not rospy.is_shutdown():
            resp = search_service('ladder_hole_3')
            hole = resp.hole

def search_ladder_fine_3(args):
    global hole
    resp = search_service('ladder_hole_fine_3')
    hole = resp.hole
    while not hole.found and not rospy.is_shutdown():
        search_ladder_3(None)
        resp = search_service('ladder_hole_fine_3')
        hole = resp.hole

def search_ladder_4(args):
    global hole
    #resp = search_service('ladder_hole_4')
    #hole = resp.hole
    hole = Hole()
    hole.found = False
    firsttime = True
    while not hole.found and not rospy.is_shutdown():
        if not firsttime:
            align_with_hole_4(None)
        firsttime = False
        resp = search_service('ladder_hole_4')
        hole = resp.hole
        while hole.found and abs(hole.position.x) > 0.003 and not rospy.is_shutdown():
            resp = search_service('ladder_hole_4')
            hole = resp.hole

def search_ladder_fine_4(args):
    global hole
    resp = search_service('ladder_hole_fine_4')
    hole = resp.hole
    while not hole.found and not rospy.is_shutdown():
        search_ladder_4(None)
        resp = search_service('ladder_hole_fine_4')
        hole = resp.hole

def search_skin(args):
    global hole
    global last_cmd_success
    while not rospy.is_shutdown():
        resp = search_service('skin_hole')
        hole = resp.hole
        if not hole.found:
            last_cmd_success = False
            return
        elif abs(hole.position.x) > 0.0015: # not aligned in x good enough.
            print 'hole found but not aligned in x %f'%(hole.position.x)
            continue
        else:
            last_cmd_success = True
            return

def prealign_holes(args):
    global hole
    global last_cmd_success
    while not rospy.is_shutdown():
        resp = search_service('prealign')
        hole = resp.hole
        if not hole.found:
            last_cmd_success = False
            print 'prealign_holes failed.'
            return
        elif abs(hole.position.x) > 0.0015: # not aligned in x good enough.
            print 'hole found but not aligned in x %f'%(hole.position.x)
            continue
        else:
            last_cmd_success = True
            return

def align_holes(args):
    global hole
    global last_cmd_success
    resp = search_service('align')
    hole = resp.hole
    last_cmd_success = hole.found

def prealign_holes2(args):
    global hole
    global last_cmd_success
    while not rospy.is_shutdown():
        resp = search_service('prealign2')
        hole = resp.hole
        if not hole.found:
            last_cmd_success = False
            print 'prealign_holes2 failed.'
            return
        elif abs(hole.position.x) > 0.0015: # not aligned in x good enough.
            print 'hole found but not aligned in x %f'%(hole.position.x)
            continue
        else:
            last_cmd_success = True
            return

def align_holes2(args):
    global hole
    global last_cmd_success
    resp = search_service('align2')
    hole = resp.hole
    last_cmd_success = hole.found

###########################################################

print "Creating cmds"

#TODO: these sorts of things should really just use reflection
cmds = { 'go_to_rack': go_to_rack,
         'go_to_drc4': go_to_drc4,
         'go_to_ladder': go_to_ladder,
         'grasp_wing_pose': grasp_wing_pose,
         'grasp_wing_side_pose': grasp_wing_side_pose,
         'arm_feedback_to_grasp_wing': arm_feedback_to_grasp_wing,
         'arm_feedback_to_grasp_wing_side': arm_feedback_to_grasp_wing_side,
         'close_gripper': close_gripper,
         'open_gripper': open_gripper,
         'compliant_grip': compliant_grip,
         'rotate_skin': rotate_skin,
         'back_away': back_away,
         'back_away_right': back_away_right,
         'circular_motion': circular_motion,
         'halt' : halt,
         'align_with_hole' : align_with_hole,
         'search_ladder' : search_ladder,
         'search_ladder_fine' : search_ladder_fine,
         'search_skin' : search_skin,
         'align_holes' : align_holes,
         'prealign_holes' : prealign_holes,
         'align_holes2' : align_holes2,
         'prealign_holes2' : prealign_holes2,
         'Insert' : Insert,
         'LiftTool': LiftTool,
         'back_away_slightly' : back_away_slightly,
         'push_pose' : push_pose,
         'slide_pose' : slide_pose,
         'arm_up_pose' : arm_up_pose,
         'push_first' : push_first,
         'push_first_backup' : push_first_backup,
         'push_second' : push_second,
         'hole1_inspection_pose' : hole1_inspection_pose,
         'hole2_inspection_pose' : hole2_inspection_pose,
         'hole3_inspection_pose' : hole3_inspection_pose,
         'hole4_inspection_pose' : hole4_inspection_pose,
         'push_second_backup' : push_second_backup,
         'align_with_hole_2': align_with_hole_2,
         'align_with_hole_3': align_with_hole_3,
         'align_with_hole_4': align_with_hole_4,
         'search_ladder_2' : search_ladder_2,
         'search_ladder_2_bottom' : search_ladder_2_bottom,
         'search_ladder_fine_2' : search_ladder_fine_2,
         'search_ladder_fine_2_bottom' : search_ladder_fine_2_bottom,
         'search_ladder_3' : search_ladder_3,
         'search_ladder_fine_3' : search_ladder_fine_3,
         'search_ladder_4' : search_ladder_4,
         'search_ladder_fine_4' : search_ladder_fine_4,
         'arm_inspection_config_tilted' : arm_inspection_config_tilted,
         'arm_inspection_config' : arm_inspection_config,
         'set_joint_5' : set_joint_5}

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
request_topic = 'wing_transport_request'
reply_topic = 'wing_transport_reply'
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

hole = None
search_service = rospy.ServiceProxy(my_name + 'start_search', SearchHole)
insert_service = rospy.ServiceProxy(my_name + 'insert', InsertToHole)
lifttool_service = rospy.ServiceProxy(my_name + 'lifttool', InsertToHole)

rospy.spin()
