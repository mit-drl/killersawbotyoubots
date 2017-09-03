#!/usr/bin/env python

import roslib; roslib.load_manifest('fleet_control')
from geometry_msgs.msg import Twist,Pose,PoseStamped,PointStamped, Point
from assembly_common.srv import BasePose,ArmCommand
from assembly_common.msg import CommunicationStamped, Communication
from mit_msgs.msg import MocapPositionArray
from fleet_control.msg import Fleet, FleetCommand
#from fleet_control.srv import CheckSuccess
from std_msgs.msg import String,Float64, Header, Time
#from brics_actuator.msg import JointVelocities
from visualization_msgs.msg import Marker
import euclid
import time
import rospy
import math
import numpy as np
import string
import IPython

import sys
import subprocess
import tf

#from transformations.py import *

from vicon_utils import *


rospy.init_node('wood_transport_master')
robot_names = ['drc1', 'drc3']

fleet_origin = None





def wait_for_subscriber(pub):
    pass
    
def get_vel(error, max_vel, gain):
    error *= gain
    if abs(error) > max_vel:
        return math.copysign(max_vel, -error)
    else:
        return -error
    
    
def get_fleet_origin_in_frame(frame):
    global_fleet_pose = PoseStamped()
    global_fleet_pose.header.frame_id = '/map'
    global_fleet_pose.pose = fleet_origin
    return transform_by_subjects(global_fleet_pose, frame)
    
def from_frame_to_fleet_origin_frame(frame, twist):
    #twist = copy.copy(twist)
    local_fleet_pose = get_fleet_origin_in_frame(frame)
    qt = euclid.Quaternion(local_fleet_pose.pose.orientation.w,
                           local_fleet_pose.pose.orientation.x,
                           local_fleet_pose.pose.orientation.y,
                           local_fleet_pose.pose.orientation.z)
    #fleet_pitch, fleet_yaw, fleet_roll = qt.get_euler()
    fleet_pitch, fleet_yaw, fleet_roll = qt.get_euler()
    
    # Mehmet added this block. TODO: Uncomment and test.
    ## First represent the twist relative to fleet_frame, but represented in frame
    #angular_vel_vector = np.array([0.0,0.0,twist.angular.z])
    #frame_to_fleet_origin_vector = np.array([local_fleet_pose.pose.position.x,local_fleet_pose.position.y,0.0])
    #linear_vel_at_fleet_due_to_angular_vel_at_frame = np.cross(angular_vel_vector,frame_to_fleet_origin_vector)
    #twist.linear.x = twist.linear.x + linear_vel_at_fleet_due_to_angular_vel_at_frame[0]
    #twist.linear.y = twist.linear.y + linear_vel_at_fleet_due_to_angular_vel_at_frame[1]
    ## Now represent everything in fleet_frame
    out = Twist()
    sy = math.sin(fleet_yaw)
    cy = math.cos(fleet_yaw)
    out.linear.x =  cy * twist.linear.x + sy * twist.linear.y
    out.linear.y = -sy * twist.linear.x + cy * twist.linear.y
    out.linear.z = twist.linear.z
    out.angular.z = twist.angular.z
    # Assume roll and pitch are zero
    return out
    
def get_multi_vel(error, max_vel, gain):
    out = list()
    ss = 0.0
    if max_vel == 0.0:
        return [ 0.0 for ii in error ]

    for item in error:
        out.append(-item * gain)
        ss += item * item

    ss = math.sqrt(ss)
    #print "\tss: %f   max_vel: %f" % (ss, max_vel)
    if ss > max_vel:
        out2 = list()
        for item in out:
            out2.append(item * max_vel / ss)
        out = out2

    #sys.stdout.write("GMV: ")
    #for item in out:
        #sys.stdout.write("%f " % item)
    #sys.stdout.write("\n")

    return out

def regrasp(dist_back_x, min_dist):
    #step 1: get the poses in the world frame\
    print 'drc1'
    drc1 = vicon_to_geometry_pose('drc1')
    print 'drc3'
    drc3 = vicon_to_geometry_pose('drc3')
    print 'woodmount1'
    woodmount1 = vicon_to_geometry_pose('woodmount1')
    print 'woodmount2'
    woodmount2 = vicon_to_geometry_pose('woodmount2')
    
    midpoint_vicon = vicon_to_geometry_pose('killer_death_sawbot')
    
    midpoint = np.array([midpoint_vicon.position.x, midpoint_vicon.position.y, midpoint_vicon.position.z])
    
    wood1_pos = woodmount1.position
    wood2_pos = woodmount2.position
    drc1_pos = drc1.position
    drc3_pos = drc3.position
    
    drc1_pos_array = np.array([drc1_pos.x, drc1_pos.y, drc1_pos.z])
    drc3_pos_array = np.array([drc3_pos.x, drc3_pos.y, drc3_pos.z])
    
    wood1_pos_array = np.array([wood1_pos.x, wood1_pos.y, wood1_pos.z])
    wood2_pos_array = np.array([wood2_pos.x, wood2_pos.y, wood2_pos.z])
    
    #midpoint = (wood1_pos_array + wood2_pos_array) / 2.0
    
    drc1_new_pos = (wood1_pos_array + midpoint) / 2.0
    drc3_new_pos = (wood2_pos_array + midpoint) / 2.0
    
    
    
    drc1_pose = geometry_pose_to_pose(drc1)
    drc3_pose = geometry_pose_to_pose(drc3)
    
    drc1_yaw = drc1_pose.get_yaw()
    drc3_yaw = drc3_pose.get_yaw()
    
    drc1_x = drc1_new_pos[0]
    drc3_x = drc3_new_pos[0]
    
    drc1_y = drc1_new_pos[1] + back_amt
    drc3_y = drc3_new_pos[1] + back_amt
    
    
    print abs(drc1_new_pos[0] - midpoint[0])
    if abs(drc1_new_pos[0] - midpoint[0]) < min_dist:
        print 'adjust 1'
        drc1_new_pos[0] = midpoint[0] - min_dist
    
    print abs(drc3_new_pos[0] - midpoint[0])
    if abs(drc3_new_pos[0] - midpoint[0]) < min_dist:
        print 'adjust 3'
        drc3_new_pos[0] = midpoint[0] + min_dist
        
    
    
    
    
    base_command1(drc1_x, drc1_y, drc1_yaw, 0.025, 0.1, False, "/map")
    base_command3(drc3_x, drc3_y, drc3_yaw, 0.025, 0.1, False, "/map")

def servo_fleet(target_pose, frame, lin_vel, ang_vel):
    epsilon = 0.009

    target_pose = geometry_pose_to_pose(target_pose)
    
    

    while not rospy.is_shutdown():
        if fleet_origin is None:
            rospy.sleep(0.5)
            continue

 
        # get pose of fleet center and transform it
        local_fleet_pose = get_fleet_origin_in_frame(frame)

        dx = local_fleet_pose.pose.position.x - target_pose.trans.x
        dy = local_fleet_pose.pose.position.y - target_pose.trans.y
        dz = local_fleet_pose.pose.position.z - target_pose.trans.z
        qt = euclid.Quaternion(local_fleet_pose.pose.orientation.w,
                               local_fleet_pose.pose.orientation.x,
                               local_fleet_pose.pose.orientation.y,
                               local_fleet_pose.pose.orientation.z)
        pitch, yaw, roll = qt.get_euler()
        #dyaw = normalize(yaw - target_pose.get_yaw())
        dyaw = normalize(yaw - target_pose.get_yaw())
        XY_GAIN  = 1
        Z_GAIN   = 0.0
        ROT_GAIN = 2
        
        #XY_GAIN = 0.1
        #ROT_GAIN = 0.0

        done = True
        #if lin_vel > 0.0 and (abs(dx) > epsilon or abs(dy) > epsilon """or abs(dz) > epsilon"""):
        if lin_vel > 0.0 and (abs(dx) > epsilon or abs(dy) > epsilon):
            done = False
        if ang_vel > 0.0 and abs(dyaw) > epsilon:
            done = False

        if done:
            print 'Donions!'
            break

        twist = Twist()
      
        twist.linear.x, twist.linear.y = get_multi_vel([dx, dy], lin_vel, XY_GAIN)
        #twist.linear.z = get_vel(dz, lin_vel/4.0, 0.0)
        twist.angular.z = get_vel(dyaw, ang_vel, ROT_GAIN)
        #print "err: %f %f %f %f   cmd: %f %f %f %f" % (dx, dy, dz, dyaw, twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)
        #IPython.embed()
        # Now we have a command; must convert it to the fleet origin frame
        twist = from_frame_to_fleet_origin_frame(frame, twist)
        #twist.angular.z *= -1
        twist.linear.y *= 1
        
        print 'dx is ', dx
        print 'dy is ', dy
        print 'dyaw is ', dyaw
        fleet_vel.publish(twist)
        rospy.sleep(0.01)

    # send stop command
    fleet_vel.publish(Twist())
    
def handle_fleet_origin(msg):
    global fleet_origin
    fleet_origin = msg
    
def multi_cmd_fleet(msg):
    fleet_vel.publish(msg)
    
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



#Handle setup of ros topics and services:

fleet_vel_topic = '/multi_cmd'
fleet_origin_topic  = '/fleet_origin'
base_service1 = '/drc1/robot_base_command'
base_service3 = '/drc3/robot_base_command'

arm_service1 = '/drc1/robot_arm_command'
arm_service3 = '/drc3/robot_arm_command'


fleet_vel = rospy.Publisher(fleet_vel_topic, Twist, latch=False)
fleet_origin_sub = rospy.Subscriber(fleet_origin_topic, Pose, handle_fleet_origin)

rospy.wait_for_service(base_service1)
rospy.wait_for_service(base_service3)
rospy.wait_for_service(arm_service1)
rospy.wait_for_service(arm_service3)


base_command1 = rospy.ServiceProxy(base_service1, BasePose)
print 'base_service1 available'
base_command3 = rospy.ServiceProxy(base_service3, BasePose)
print 'base_service3 available'
arm_command1 = rospy.ServiceProxy(arm_service1, ArmCommand)
print 'arm service 1 available'
arm_command3 = rospy.ServiceProxy(arm_service3, ArmCommand)
print 'arm service 3 available'



#Step 1: arm position


rospy.sleep(1.0)
#Step 2: youbot location


print 'time to get into position'

arm_command1('post_grab_lumber')
print 'arm 1 done'
arm_command3('post_grab_lumber')
print 'arm 3 done'

base_command1(0.0, 0.45, -math.pi, 0.1, 0.1, False, "/woodmount1")
base_command3(0.0, -0.45, -math.pi, 0.1, 0.1, False, "/woodmount2")

arm_command1('pre_grab_lumber')
arm_command3('pre_grab_lumber')

print 'done moving bases'


#Step 3: set fleet
set_fleet = rospy.Publisher('/set_fleet', Fleet, latch=True)



fleet = Fleet()
fleet.group.extend(robot_names)
wait_for_subscriber(set_fleet)
print fleet
set_fleet.publish(fleet) #Step 1

#Step 4: move to saw

pose = Pose()


pose.orientation = Quaternion.new_rotate_euler(0.0, 0.0, 0.0)

back_amt = -0.3
pose.position.x = back_amt
pose.position.y = 0.0
pose.position.z = 0.0


frame = '/killer_death_sawbot'
lin_vel = 0.1
ang_vel = 0.2

rospy.sleep(2.0)


#print 'test linear velocity'
#twist = Twist()
#twist.linear.x = 0.01
#twist.angular.z = 0.1
#multi_cmd_fleet(twist)

raw_input('hello')
print 'servo fleet time'
arm_command1('saw_pose')
arm_command3('saw_pose')
servo_fleet(pose, frame, lin_vel, ang_vel) #Step 2

fleet = Fleet()
print fleet
set_fleet.publish(fleet)

#Step 6: regrasp

safety_thresh = 0.9
raw_input('are you ready?')
regrasp(back_amt, safety_thresh) #backward amount, safety threshold

raw_input('sawwww')
#Step 5: break the fleet:



print 'done moving everything'
arm_command1('post_grab_lumber')
arm_command3('post_grab_lumber')



rospy.spin()
