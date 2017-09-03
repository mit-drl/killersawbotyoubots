#!/usr/bin/env python

## This node directs two robots transporting a wing skin from a start to
## a goal location.  The start location is on a rack, and the goal is
## mounted on the wing ladder.

import roslib; roslib.load_manifest('fleet_control')
from geometry_msgs.msg import Twist,Pose,PoseStamped,PointStamped, Point
from assembly_common.srv import BasePose,ArmCommand
from assembly_common.msg import CommunicationStamped, Communication
from visual_inspection.srv import *
from mit_msgs.msg import MocapPositionArray
from track_obj.msg import StartTrackingGPU
#from track_obj.msg import StartTracking
from fleet_control.msg import Fleet, FleetCommand
from fleet_control.srv import CheckSuccess
from std_msgs.msg import String,Float64, Header, Time
from brics_actuator.msg import JointVelocities
from visualization_msgs.msg import Marker
import euclid
import time
import rospy
import math
import numpy as np
import string

import sys
import subprocess
import tf
import speech_recognition as sr


robot = dict()
robot['carry'] = ['drc3', 'drc1']
robot['coarse_perception'] = ['drc4']
robot['fine_perception'] = ['drc2']
robot['pushing_robot'] = ['drc1']
robot['master'] = ['master']

robot_to_id = dict()
robot_to_id['master'] = 0
robot_to_id['drc1'] = 1
robot_to_id['drc2'] = 2
robot_to_id['drc3'] = 3
robot_to_id['drc4'] = 4

usehokuyo = True
usekinect = True
reallyusekinect = False
usevisinsp = True
new_2nd_hole_alignment = True
skewed = 0.0
voice = True

all_robots = ['drc3', 'drc2', 'drc1', 'drc4', 'master']
#all_robots = ['drc3', 'drc1', 'drc2', 'master']


pregrasp_radius = 1.345
lidar_orientation = np.pi/2
grasp_radius = 0.335 ## from hand to center
grasp_radius_intermediate = 0.380

prepush_radius = 1.345 #1.045

tracker_topic = '/tracker_request'
ack = None
last_cmd = None #dict()
fleet_origin = None

rospy.init_node('wing_transport_master')

from vicon_utils import *
import kinect_utils

camera_robot = 'drc2'
camera_poses = {
    'away': ['/ladder', -1.8, 0.0, 0.0],
    'view_ladder': ['/ladder',-0.928, 0.671, -0.368],
    'inspect_ladder': ['/ladder', -1.110, -0.072, 0.4]
}

pub_start_track = rospy.Publisher('/start_track', StartTrackingGPU)
pub_stop_track = rospy.Publisher('/stop_track', String)

#XY_GAIN  = 1
XY_GAIN  = 0.2
#Z_GAIN   = 1
Z_GAIN   = 0.2
#ROT_GAIN = 2
ROT_GAIN = 0.8

track_skin = False

##################################################################

def set_view(name):
    view_pub.publish(String(name))

def get_input():
    r = sr.Recognizer()
    with sr.Microphone() as source: # use the default microphone as the audio source
        audio = r.listen(source) # listen for the first phrase and extract it into audio data

        try:
            text = string.lower(r.recognize(audio)) # recognize speech using Google Speech Recognition
            return text
        except LookupError: # speech is unintelligible
            print("Could not understand audio")
            return ""
            
def wait_for(text):
    print "Waiting for input of " + text
    while True:
        input_text = get_input()
        if input_text == 'stop':
            exit() #special case
        if input_text == text:
            break

# P-controller with bounded velocity
def get_vel(error, max_vel, gain):
    error *= gain
    if abs(error) > max_vel:
        return math.copysign(max_vel, -error)
    else:
        return -error


# P-controller with bounded velocity
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


def set_camera_pose(info):
    frame, xx, yy, th = info
    camera_base_service(xx, yy, th, 0.03, 0.1, False, frame)
    #camera_arm_service(pose)


def servo_fleet(target_pose, frame, lin_vel, ang_vel):
    global track_skin
    epsilon = 0.009

    target_pose = geometry_pose_to_pose(target_pose)

    while not rospy.is_shutdown():
        if fleet_origin is None:
            rospy.sleep(0.5)
            continue

        if track_skin:
            skin_in_ladder = get_skin_in_ladder_kinect()
            #print "skin in ladder:   x: %f,   y: %f,   z: %f,   yaw: %f"%( skin_in_ladder[3], skin_in_ladder[7], skin_in_ladder[11], np.arctan2(skin_in_ladder[4],skin_in_ladder[0]))
            dx = skin_in_ladder[3]# - 0.02 # 2cm overshoot for hole search
            dy = skin_in_ladder[7]# - 0.02 # 2cm overshoot for hole search
            dz = skin_in_ladder[11] - 0.15
            dyaw = np.arctan2(skin_in_ladder[4],skin_in_ladder[0])
            XY_GAIN  = 0.2
            Z_GAIN   = 0.2
            ROT_GAIN = 0.8
        else:
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
            dyaw = normalize(yaw - target_pose.get_yaw())
            XY_GAIN  = 1
            Z_GAIN   = 1
            ROT_GAIN = 2

        done = True
        if lin_vel > 0.0 and (abs(dx) > epsilon or abs(dy) > epsilon or abs(dz) > epsilon):
            done = False
        if ang_vel > 0.0 and abs(dyaw) > epsilon:
            done = False

        if done:
            if track_skin:
                track_skin = False
            print 'Donions!'
            break

        twist = Twist()
        twist.linear.x, twist.linear.y = get_multi_vel([dx, dy], lin_vel, XY_GAIN)
        twist.linear.z = get_vel(dz, lin_vel/4.0, Z_GAIN)
        twist.angular.z = get_vel(dyaw, ang_vel, ROT_GAIN)
        #print "err: %f %f %f %f   cmd: %f %f %f %f" % (dx, dy, dz, dyaw, twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)

        # Now we have a command; must convert it to the fleet origin frame
        twist = from_frame_to_fleet_origin_frame(frame, twist)
        #print "\tin fof: %f %f" % (twist.linear.x, twist.linear.y)
        #print 'twist is'
        #print twist
        fleet_vel.publish(twist)
        rospy.sleep(0.01)

    # send stop command
    fleet_vel.publish(Twist())


##################################################################

def lift_wing_skin():
    # Cartesian control via fleet control
    pose = Pose()
    pose.position.z = 0.10
    pose.orientation.w = 1.0
    frame = '/nu_rack'
    lin_vel = 0.05
    ang_vel = 0.0
    servo_fleet(pose, frame, lin_vel, ang_vel)

def nudge_skin():
    # Cartesian control via fleet control
    frame = '/nu_rack'
    pose = get_fleet_origin_in_frame(frame)
    pose.pose.position.y += 0.04
    lin_vel = 0.05
    ang_vel = 0.0
    servo_fleet(pose.pose, frame, lin_vel, ang_vel)

def fix_grasp():
    # Cartesian control via fleet control
    frame = '/nu_rack'
    pose = get_fleet_origin_in_frame(frame)
    pose.pose.position.z -= 0.005
    lin_vel = 0.05
    ang_vel = 0.0
    servo_fleet(pose.pose, frame, lin_vel, ang_vel)

def tuck_arms():
    fcmd = FleetCommand()
    fcmd.command = 'tuck_arms'
    fcmd.data = 6.5 # tuck arms for 6.5 sec
    fleet_cmd_pub.publish(fcmd)
    #time.sleep(duration.data)
    #time.sleep(1.0)

def pause_fleet():
    fcmd = FleetCommand()
    fcmd.command = 'pause'
    fleet_cmd_pub.publish(fcmd)

def resume_fleet():
    fcmd = FleetCommand()
    fcmd.command = 'resume'
    fleet_cmd_pub.publish(fcmd)

def lower_wing_skin():
    # Fleet control command
    pose = Pose() # Target is near origin
    pose.position.z = 0.07
    rot = Quaternion.new_rotate_euler(0, orig_fleet_yaw, 0)
    pose.orientation.x = rot.x
    pose.orientation.y = rot.y
    pose.orientation.z = rot.z
    pose.orientation.w = rot.w
    frame = '/ladder'
    lin_vel = 0.05
    ang_vel = 0.05
    servo_fleet(pose, frame, lin_vel, ang_vel)


def lower_wing_skin_1():
    # Fleet control command
    fleet_pose = get_fleet_origin_in_frame('/ladder')
    #fleet_pose.pose.position.z = fleet_pose.pose.position.z - 0.04
    fleet_pose.pose.position.z = 0.08
    # or try getting it on /map and setting to 0.24
    frame = '/ladder'
    lin_vel = 0.05
    ang_vel = 0.05
    servo_fleet(fleet_pose.pose, frame, lin_vel, ang_vel)


def lower_wing_skin_2():
    # Fleet control command
    fleet_pose = get_fleet_origin_in_frame('/ladder')
    #fleet_pose.pose.position.z = fleet_pose.pose.position.z - 0.04
    #fleet_pose.pose.position.z = 0.063
    fleet_pose.pose.position.z = 0.060
    # or try getting it on /map and setting to 0.24
    frame = '/ladder'
    lin_vel = 0.05
    ang_vel = 0.05
    servo_fleet(fleet_pose.pose, frame, lin_vel, ang_vel)

def lower_wing_skin_3():
    # Fleet control command
    fleet_pose = get_fleet_origin_in_frame('/ladder')
    #fleet_pose.pose.position.z = fleet_pose.pose.position.z - 0.04
    #fleet_pose.pose.position.z = 0.063
    fleet_pose.pose.position.z = 0.055
    # or try getting it on /map and setting to 0.24
    frame = '/ladder'
    lin_vel = 0.05
    ang_vel = 0.05
    servo_fleet(fleet_pose.pose, frame, lin_vel, ang_vel)

def get_skin_in_ladder_kinect():
    #ladder_in_kinect = kinect_utils.get_subject_pose('ladder')
    #skin_in_kinect = kinect_utils.get_subject_pose('skin')
    #skin_in_ladder = ladder_in_kinect.get_matrix().inverse() * skin_in_kinect.get_matrix()
    ladder_in_kinect = kinect_utils.get_subject_matrix('ladder')
    skin_in_kinect = kinect_utils.get_subject_matrix('skin')
    skin_in_ladder = skin_in_kinect * ladder_in_kinect.inverse() 

    #print "ladder in kinect"
    #print ladder_in_kinect
    #print "skin in kinect"
    #print skin_in_kinect
    #print "skin_in_ladder"
    #print skin_in_ladder

    return skin_in_ladder

def lower_wing_skin_with_kinect():
    global track_skin
    track_skin = True
    print 'lowering with kinect'
    pose = Pose() # Target is near origin
    frame = '/ladder'
    lin_vel = 0.05
    ang_vel = 0.05
    servo_fleet(pose, frame, lin_vel, ang_vel)


def visual_inspection():
    ps = PoseStamped()
    ps.header.frame_id = '/ladder'
    ps = transform_by_subjects(ps, '/kinectdrl')
    ps.pose # is the pose of the ladder in kinect according to vicon.


def rotate_to_face_ladder():
    # Fleet control command
    global orig_robot0, orig_fleet_yaw

    ps = PointStamped()
    ps.header.frame_id = '/ladder'
    ps = transform_by_subjects(ps, '/nu_rack')
    ladder = math.atan2(ps.point.y, ps.point.x) 

    ps = PointStamped()
    ps.header.frame_id = '/' + robot['carry'][0]
    ps = transform_by_subjects(ps, '/nu_rack')
    robot0 = math.atan2(ps.point.y, ps.point.x) 
    orig_robot0 = robot0

    local_fleet_pose = get_fleet_origin_in_frame('/nu_rack')
    qt = euclid.Quaternion(local_fleet_pose.pose.orientation.w,
                           local_fleet_pose.pose.orientation.x,
                           local_fleet_pose.pose.orientation.y,
                           local_fleet_pose.pose.orientation.z)
    fleet_pitch, fleet_yaw, fleet_roll = qt.get_euler()
    orig_fleet_yaw = fleet_yaw

    dth0 = normalize(ladder - (robot0 + 0.5 * math.pi) + fleet_yaw)
    dth1 = normalize(ladder - (robot0 - 0.5 * math.pi) + fleet_yaw)

    dyaw = dth0 if abs(normalize(dth0-fleet_yaw)) < abs(normalize(dth1-fleet_yaw)) else dth1
    # dyaw = dth0 if abs(dth0) < abs(dth1) else dth1
    #print "dyaw: %f   ladder: %f   robot0: %f   fleet_yaw: %f" % (dyaw, ladder, robot0, fleet_yaw)
    qt = Quaternion.new_rotate_euler(0.0, dyaw, 0.0)

    pose = Pose()
    pose.orientation.x = qt.x
    pose.orientation.y = qt.y
    pose.orientation.z = qt.z
    pose.orientation.w = qt.w
    frame = '/nu_rack'
    lin_vel = 0.0
    ang_vel = 0.2
    servo_fleet(pose, frame, lin_vel, ang_vel)


def translate_to_ladder():
    # Fleet control command
    pose = Pose()
    pose.position.z = 0.11
    #pose.position.x = -0.04
    pose.position.x = 0.01
    pose.orientation.w = 1.0
    frame = '/ladder'
    lin_vel = 0.1
    ang_vel = 0.0
    servo_fleet(pose, frame, lin_vel, ang_vel)


def rotate_into_ladder_alignment():
    # Fleet control command
    ps = PointStamped()
    ps.header.frame_id = '/' + robot['carry'][0]
    ps = transform_by_subjects(ps, '/ladder')
    robot0 = math.atan2(ps.point.y, ps.point.x) 

    dyaw = orig_robot0 - robot0

    local_fleet_pose = get_fleet_origin_in_frame('/ladder')
    qt = euclid.Quaternion(local_fleet_pose.pose.orientation.w,
                           local_fleet_pose.pose.orientation.x,
                           local_fleet_pose.pose.orientation.y,
                           local_fleet_pose.pose.orientation.z)
    fleet_pitch, fleet_yaw, fleet_roll = qt.get_euler()
    #print "dyaw: %f = %f - %f" % (dyaw, orig_robot0, robot0)
    #print "yaw: %f = %f - %f" % (dyaw - fleet_yaw, dyaw, fleet_yaw)
    adjust_for_grasp_error = np.pi/36.0
    if new_2nd_hole_alignment:
        # intentionally off the yaw angle so that the pannel doesn't occlude the second hole
        yaw_offset = np.pi/360.0*skewed
        wing_radius = 0.35
        #wing_radius = 0.70
        qt = Quaternion.new_rotate_euler(0.0, dyaw + fleet_yaw + adjust_for_grasp_error + yaw_offset, 0.0)
        # pose.position.y -= y_offset
    else:
        qt = Quaternion.new_rotate_euler(0.0, dyaw + fleet_yaw + adjust_for_grasp_error, 0.0)

    #qt = Quaternion.new_rotate_euler(fleet_pitch, -fleet_yaw, fleet_roll)

    pose = Pose() # Target is origin (rotation only)
    pose.orientation.x = qt.x
    pose.orientation.y = qt.y
    pose.orientation.z = qt.z
    pose.orientation.w = qt.w
    frame = '/ladder'
    lin_vel = 0.0
    ang_vel = 0.2
    servo_fleet(pose, frame, lin_vel, ang_vel)


def move_camera_robot_away():
    set_camera_pose(camera_poses['away'])

def move_camera_robot_to_view_ladder():
    set_camera_pose(camera_poses['view_ladder'])

def move_camera_robot_to_inspect_ladder():
    set_camera_pose(camera_poses['inspect_ladder'])


#def start_visual_tracking():
#    # Visual tracker command
#    msg = StartTracking()
#    msg.object_name = 'wing_skin_upper'
#
#    # Have fleet origin pose; need to compute wing skin pose from it
#    pose = geometry_pose_to_pose(fleet_origin)
#    rot = Quaternion.new_rotate_euler(0, -orig_fleet_yaw, 0)
#    pose.rot *= rot
#    msg.pose_estimate = pose_to_geometry_pose(pose)
#    tracker_pub.publish(msg)
#
#
#def stop_visual_tracking():
#    # Visual tracker command
#    msg = StartTracking() # Empty object_name means stop tracking
#
#    ## NOTE: keep this -- commented out for demo
#    #tracker_pub.publish(msg)


def visual_servo_alignment():
    # Doesn't exist yet?  Part feedback?
    pass


def establish_fleet_formation():
    fleet = Fleet()
    fleet.group.extend(robot['carry'])
    set_fleet.publish(fleet)
    origin = PoseStamped()
    origin.header.frame_id = '/map'
    origin.pose = vicon_to_geometry_pose('/nu_rack')
    transform_origin_pub.publish(origin)

def establish_fleet_formation_2():
    fleet = Fleet()
    fleet.group.extend(robot['carry'])
    set_fleet.publish(fleet)
    origin = PoseStamped()
    origin.header.frame_id = '/map'
    origin.pose = vicon_to_geometry_pose('drc4_arm')
    transform_origin_pub.publish(origin)
    
def move_to_hole_guess():
    # Fleet control command
    pose = Pose()
    pose.position.z = 0.16
    pose.orientation.w = 1.0
    frame = '/drc4_arm'
    lin_vel = 0.1
    ang_vel = 0.0
    servo_fleet(pose, frame, lin_vel, ang_vel)
    servo_fleet(pose, frame, lin_vel, ang_vel)
    
    
def fleet_search():
    pass #TODO


def break_fleet_formation():
    fleet = Fleet() # Empty fleet means break formation
    set_fleet.publish(fleet)

def start_tracking_ladder():
    init_pose_ladder = kinect_utils.get_subject_in_kinectoptical(kinectvicon_in_vicon=get_subject_transform('kinectdrl'), subject_in_vicon=get_subject_transform('ladder'))
    msg = StartTrackingGPU()
    msg.name = 'ladder'
    msg.filename = 'wing_box_ladder.ply'
    msg.x00 = init_pose_ladder.a
    msg.x01 = init_pose_ladder.b
    msg.x02 = init_pose_ladder.c
    msg.x03 = init_pose_ladder.d
    msg.x10 = init_pose_ladder.e
    msg.x11 = init_pose_ladder.f
    msg.x12 = init_pose_ladder.g
    msg.x13 = init_pose_ladder.h
    msg.x20 = init_pose_ladder.i
    msg.x21 = init_pose_ladder.j
    msg.x22 = init_pose_ladder.k
    msg.x23 = init_pose_ladder.l
    msg.x30 = init_pose_ladder.m
    msg.x31 = init_pose_ladder.n
    msg.x32 = init_pose_ladder.o
    msg.x33 = init_pose_ladder.p
    
    pub_start_track.publish(msg)

def stop_tracking_ladder():
    pub_stop_track.publish(String(data='stop'))

def start_tracking_skin():
    fleet_origin_ps = get_fleet_origin_in_frame('map')
    fleet_origin_mat = geometry_pose_to_pose(fleet_origin_ps).get_matrix()
    init_pose_skin = kinect_utils.get_subject_in_kinectoptical(kinectvicon_in_vicon=get_subject_transform('kinectdrl'), subject_in_vicon=fleet_origin_mat)
    msg = StartTrackingGPU()
    msg.name = 'skin'
    msg.filename = 'wing_skin_upper.ply'
    msg.x00 = init_pose_skin.a
    msg.x01 = init_pose_skin.b
    msg.x02 = init_pose_skin.c
    msg.x03 = init_pose_skin.d
    msg.x10 = init_pose_skin.e
    msg.x11 = init_pose_skin.f
    msg.x12 = init_pose_skin.g
    msg.x13 = init_pose_skin.h
    msg.x20 = init_pose_skin.i
    msg.x21 = init_pose_skin.j
    msg.x22 = init_pose_skin.k
    msg.x23 = init_pose_skin.l
    msg.x30 = init_pose_skin.m
    msg.x31 = init_pose_skin.n
    msg.x32 = init_pose_skin.o
    msg.x33 = init_pose_skin.p
    
    pub_start_track.publish(msg)

def publish_ring(rr, duration):
    print 'and thus we are sending'
    print robot
    if rr == 'master':
        communication_pub.publish(CommunicationStamped( 
                            header=Header(frame_id='map'),
                           comm=Communication(position=Point(2.0, -5.0, 1.0), source_name=String(data=rr), duration=Float64(data=duration))
                            ))
    else:
        mp = get_subject(rr)
        communication_pub.publish(CommunicationStamped( 
                            header=Header(frame_id='map'),
                           comm=Communication(position=Point(x=mp.translational.x / 1000.0, y=mp.translational.y / 1000.0, z=1.0), source_name=String(data=rr), duration=Float64(data=duration))
                            ))

###########################################################


def handle_fleet_origin(msg):
    global fleet_origin
    fleet_origin = msg


class Callback:
    def __init__(self, robot):
        self.robot = robot
        self.ack = False

    def callback(self, msg):
        global last_cmd
        print 'got ', msg.data
        if msg.data == last_cmd:
            print "Got ack %s for robot %s" % (msg.data, self.robot)
            self.ack = True

    def init_ack(self):
        self.ack = False


class AckHandler:
    def __init__(self, robots):
        self.robots = robots
        self.callback = dict()
        for rr in robots:
            self.callback[rr] = Callback(rr)

    #TODO: fix this
    def init_acks(self):
        global last_cmd
        print "Given new command %s" % (last_cmd)
        for rr in self.robots:
            #print self.callback[rr]
            self.callback[rr].init_ack()


    def wait_for_all_acks(self, bubbles):
        global last_cmd
        ring_count = 0
        while not rospy.is_shutdown():
            ring_count += 1
            done = True
            for rr in self.robots:
                ##############VIZ###########
                if bubbles:
                    if ring_count % 3 == 1:
                        publish_ring(rr, 1.0)
                ############################
                if False == self.callback[rr].ack:
                    print "Missing ack %s for robot %s" % (last_cmd, rr)
                    done = False
                    break
            if done:
                break
            rospy.sleep(0.5)

############################################
#AckHandler initializations
ack_carry0 = AckHandler([robot['carry'][0]])
ack_carry1 = AckHandler([robot['carry'][1]])
ack_carry = AckHandler(robot['carry'])
if usehokuyo:
    ack_fine = AckHandler(robot['fine_perception'])
if usekinect:
    pass
    #ack_coarse = AckHandler(robot['coarse_perception'])
ack_push = AckHandler(robot['pushing_robot'])
############################################



def publish_marker(speech, robot):
    marker = Marker()
    marker.action = marker.ADD
    marker.id = robot_to_id[robot]
    marker.type = marker.TEXT_VIEW_FACING
    marker.header.frame_id = '/map'
    marker.text = speech
    print 'before getting subject'

    print 'after getting subject'
    if robot == 'master':
        marker.pose.position = Point(2.0, -5.0, 0.5)
    else:
        mp = get_subject(robot)
        marker.pose.position = Point(mp.translational.x / 1000.0, mp.translational.y / 1000.0, 0.5) 
    marker.pose.orientation.w = 1.0
    marker.color.r = 0.85
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.lifetime.secs = 4.0
    text_pub.publish(marker)

    print 'published text!'

def distributed_cmd(cmd, robot=robot['carry'], ack=ack_carry, speech=None, pause=False, duration=5.0, bubbles=True):
    start = time.time()
    global last_cmd
    # Send command
    print "DISTRIBUTED %s" % (cmd)
    print speech
    if speech is not None:
        for rr in robot:
            publish_marker(speech, rr)
            if rr == 'master':
                text_to_speech[rr].publish(speech)
            else:
                text_to_speech_male[rr].publish(speech)
    
        
    last_cmd = cmd
    ack.init_acks()

    for rr in robot:
        msg = String()
        msg.data = cmd
        pub[rr].publish(msg)

    # Wait for replies
    ack.wait_for_all_acks(bubbles)

    if pause:
        print("Press return to continue.")
        #raw_input("Done.\n")
    return time.time() - start


def distributed_insert_cmd(cmd, robot=robot['carry'], ack=ack_carry, angle=0.0, hole_name='', speech=None, pause=False, duration=5.0, bubbles=True):
    global last_cmd
    # Send command
    print "DISTRIBUTED %s" % (cmd)
    print speech
    if speech is not None:
        for rr in robot:
            publish_marker(speech, rr)
            if rr == 'master':
                text_to_speech[rr].publish(speech)
            else:
                text_to_speech_male[rr].publish(speech)
    
        
    last_cmd = cmd
    ack.init_acks()

    for rr in robot:
        msg = String()
        msg.data = cmd + ' ' + str(angle) + ' ' + hole_name
        pub[rr].publish(msg)

    # Wait for replies
    ack.wait_for_all_acks(bubbles)


def distributed_joint_cmd(cmd, robot=robot['carry'], ack=ack_carry, angle=0.0, speech=None, pause=False, duration=5., bubbles=True):
    global last_cmd
    # Send command
    print "DISTRIBUTED %s" % (cmd)
    print speech
    if speech is not None:
        for rr in robot:
            publish_marker(speech, rr)
            if rr == 'master':
                text_to_speech[rr].publish(speech)
            else:
                text_to_speech_male[rr].publish(speech)
    
        
    last_cmd = cmd
    ack.init_acks()

    for rr in robot:
        msg = String()
        msg.data = cmd + ' ' + str(angle)
        pub[rr].publish(msg)

    # Wait for replies
    ack.wait_for_all_acks(bubbles)


def distributed_base_cmd(base_cmd, angle0, radius, robot=robot['carry'], ack=ack_carry, speech=None, pause=False, duration=5.0, bubbles=True):
    # Add args to the command giving base position and orientation,
    # which must be different for each robot.  The robots are assumed
    # to form a circle in the target reference frame, each pointing
    # toward the frame origin.  The first robot is located at a heading
    # angle0 with respect to the target frame.

    global last_cmd

    print "DISTRIBUTED BASE %s" % (base_cmd)

    if speech is not None:
        for rr in robot:
            publish_marker(speech, rr)
            if rr == 'master':
                text_to_speech[rr].publish(speech)
            else:
                text_to_speech_male[rr].publish(speech)

    angle = angle0
    angle_incr = 2.0 * math.pi / len(robot)
    last_cmd = base_cmd
    ack.init_acks()

    for rr in robot:
        xx = radius * math.cos(angle)
        yy = radius * math.sin(angle)
        theta = math.pi + angle
        cmd = '%s %f %f %f' % (base_cmd, xx, yy, theta)

        msg = String()
        msg.data = cmd
        pub[rr].publish(msg)

        angle += angle_incr

    # Wait for replies
        

    ack.wait_for_all_acks(bubbles)

    if pause:
        print("Press return to continue.")
        #raw_input("Done.\n")
        

        



cmds = { 'lift_wing_skin': lift_wing_skin,
         'lower_wing_skin': lower_wing_skin_with_kinect,
         'lower_wing_skin_1': lower_wing_skin_1,
         'lower_wing_skin_2': lower_wing_skin_2,
         'lower_wing_skin_3': lower_wing_skin_3,
         'rotate_to_face_ladder': rotate_to_face_ladder,
         'fix_grasp': fix_grasp,
         'translate_to_ladder': translate_to_ladder,
         'rotate_into_ladder_alignment': rotate_into_ladder_alignment,
         'move_camera_robot_away': move_camera_robot_away, 
         'move_camera_robot_to_view_ladder': move_camera_robot_to_view_ladder, 
         'move_camera_robot_to_inspect_ladder': move_camera_robot_to_inspect_ladder, 
         # 'move_camera_robot_to_side': move_camera_robot_to_side,
         # 'move_camera_to_view_ladder': move_to_camera_pose,
         #'start_visual_tracking': start_visual_tracking,
         'visual_servo_alignment': visual_servo_alignment,
         #'stop_visual_tracking': stop_visual_tracking,
         'establish_fleet_formation': establish_fleet_formation,
         'tuck_arms': tuck_arms,
         'pause_fleet': pause_fleet,
         'resume_fleet': resume_fleet,
         'break_fleet_formation': break_fleet_formation,
         'establish_fleet_formation_2' : establish_fleet_formation_2,
         'move_to_hole_guess' : move_to_hole_guess,
         'fleet_search' : fleet_search,
         'start_tracking_ladder' : start_tracking_ladder,
         'stop_tracking_ladder' : stop_tracking_ladder,
         'start_tracking_skin' : start_tracking_skin,
         'nudge_skin' : nudge_skin}


def centralized_cmd(cmd, robot=robot['carry'], speech=None, pause=False, duration=5.0):
    start = time.time()
    print "I'm publishing..."
    print robot
    print "CENTRALIZED command: %s" % cmd
    if speech is not None:
        for rr in robot:
            publish_ring(rr, 10.0)
        for rr in robot:
            publish_marker(speech, rr)
            if rr == 'master':
                text_to_speech[rr].publish(speech)
                
            else:
                text_to_speech_male[rr].publish(speech)
                
            

    cmds[cmd]()

    if pause:
        print("Press return to continue.")
        #raw_input("Done.\n")
    return time.time() - start

def print_ladder_skin_offset():
    skinpose = vicon_to_geometry_pose('/skin')
    skinpose_ps = PoseStamped()
    skinpose_ps.header.frame_id = '/map'
    skin_in_ladder = transform_by_subjects(skinpose_ps,'/ladder')
    qt = euclid.Quaternion(skin_in_ladder.pose.orientation.w,
                           skin_in_ladder.pose.orientation.x,
                           skin_in_ladder.pose.orientation.y,
                           skin_in_ladder.pose.orientation.z)
    skin_pitch, skin_yaw, skin_roll = qt.get_euler()
    print 'SKIN IN LADDER: x: %f    y: %f    yaw: %f'%(skin_in_ladder.pose.position.x,
                                                       skin_in_ladder.pose.position.y,
                                                       skin_yaw)


# visual inspection 

# def visual_inspection(objectname, kinectname):
#     ps = PoseStamped()
#     ps.header.frame_id = objectname
#     ps = transform_by_subjects(ps, kinectname)
#     return ps.pose # is the pose of the ladder in kinect according to vicon.

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

def run_visinsp(target):
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








# Step 0) Initialization

pub = dict()
sub = dict()
sub_individual = dict()
stop = dict()
text_to_speech = dict()
text_to_speech_male = dict()
last_cmd = 'init'
ack_carry.init_acks() # TODO: There is a race condition: this might run second, thus wait forever
view_pub = rospy.Publisher("/current_frame", String)

if usehokuyo:
    ack_fine.init_acks() #TODO: eventually incorporate this into cleco movement.
if usekinect:
    pass
    #ack_coarse.init_acks()
ack_push.init_acks()

for rr in all_robots:
    pub_topic = '/' + rr + '/wing_transport_request'
    sub_topic = '/' + rr + '/wing_transport_reply'
    speech_topic = '/' + rr + '/text_to_speech'
    speech_topic_male = '/' + rr + '/text_to_speech_male'
    stop_topic = '/' + rr + '/arm_1/arm_controller/velocity_command'
    # TODO: rr is lazily evaluated in lambda, so this shouldn't work!
    if rr == robot['carry'][0]:
        sub_individual[rr] = rospy.Subscriber(sub_topic, String, ack_carry0.callback[rr].callback)
    elif rr == robot['carry'][1]:
        sub_individual[rr] = rospy.Subscriber(sub_topic, String, ack_carry1.callback[rr].callback)

    if rr in robot['carry']:
        sub[rr] = rospy.Subscriber(sub_topic, String, ack_carry.callback[rr].callback)
    if usehokuyo and rr in robot['fine_perception']:
        sub[rr] = rospy.Subscriber(sub_topic, String, ack_fine.callback[rr].callback)

    #TODO: how do I handle a robot with multiple roles?
    if rr in robot['pushing_robot']:
        sub[rr] = rospy.Subscriber(sub_topic, String, ack_push.callback[rr].callback)

    if usekinect and rr in robot['coarse_perception']:
        pass
        #sub[rr] = rospy.Subscriber(sub_topic, String, ack_coarse.callback[rr].callback)
    
    pub[rr] = rospy.Publisher(pub_topic, String)
    stop[rr] = rospy.Publisher(stop_topic, JointVelocities)
    text_to_speech[rr] = rospy.Publisher(speech_topic, String)
    text_to_speech_male[rr] = rospy.Publisher(speech_topic_male, String)

text_pub = rospy.Publisher('/speech_text', Marker)
communication_pub = rospy.Publisher('/communication', CommunicationStamped, latch=True)
    



# For camera robot
#camera_base = dict()
camera_base_service = None
#camera_arm = dict()

if usekinect:
    camera_base_service = rospy.ServiceProxy('/' + robot['coarse_perception'][0] + '/robot_base_command', BasePose)
    #camera_arm[rr]  = rospy.ServiceProxy('/' + rr + '/robot_arm_command', ArmCommand)



tracker_topic = '/tracker_request'
set_fleet_topic = '/set_fleet' 
fleet_vel_topic = '/multi_cmd'
fleet_cmd_topic = '/fleet_cmd'
transform_origin_topic = '/transform_origin'
fleet_origin_topic  = '/fleet_origin'


#tracker_pub = rospy.Publisher(tracker_topic, StartTracking, latch=True)
set_fleet = rospy.Publisher(set_fleet_topic, Fleet, latch=True)
fleet_vel = rospy.Publisher(fleet_vel_topic, Twist, latch=False)
fleet_cmd_pub = rospy.Publisher(fleet_cmd_topic, FleetCommand, latch=False)
transform_origin_pub = rospy.Publisher(transform_origin_topic, PoseStamped, latch=True)
fleet_origin_sub = rospy.Subscriber(fleet_origin_topic, Pose, handle_fleet_origin)

check_success={}
check_success[robot['fine_perception'][0]]  = rospy.ServiceProxy('/'+robot['fine_perception'][0]+'/did_last_command_succeed', CheckSuccess)


if usehokuyo:
    ack_fine.wait_for_all_acks(True)
if usekinect:
    pass
    #ack_coarse.wait_for_all_acks(True)

ack_carry.wait_for_all_acks(True)
print "Finished pub/sub"

#TODO: Andy new sequence of comamnds
#distributed_base_cmd('establish_fleet_formation_2')
#distributed_base_cmd('fleet_search')

do_first_hole = True

starttime = time.time()
lasttime = time.time()


#Timing stuff

hole_detection_one_acc = 0
hole_detection_carry_acc = 0
hole_detection_other_acc = 0
vicon_acc = 0
errors = -1

raw_input('hit enter to start demo')

try:
  set_view('default')
  distributed_cmd('open_gripper')
  if do_first_hole:
      if usekinect:
        centralized_cmd('move_camera_robot_to_view_ladder', 'move camera robot to view ladder')
        centralized_cmd('stop_tracking_ladder', robot['master'], 'All Systems are activated and ready to perform assembly task.')
        time.sleep(1.0)
        centralized_cmd('start_tracking_ladder')
        time.sleep(1.0)
        centralized_cmd('stop_tracking_ladder')
        time.sleep(1.0)
        centralized_cmd('start_tracking_ladder')
        # centralized_cmd('move_camera_to_view_ladder', 'Move into position')

      rospy.sleep(2.0) #To avoid speech overlay
      ###distributed_base_cmd('go_to_ladder', np.pi/4.0, pregrasp_radius, robot['fine_perception'], ack_fine, 'Robot 4 moving into position to find first hole.', duration=5.0, bubbles=False)

      if voice:
          wait_for('find the hole')
          text_to_speech['master'].publish('Finding the hole.')
      vicon_acc += distributed_cmd('align_with_hole', robot['fine_perception'], ack_fine, duration=10.0)
      
      if usehokuyo:
          #raw_input('Hit Enter to search hole')
          hole_detection_one_acc += distributed_cmd('search_ladder', robot['fine_perception'], ack_fine, 'Robot 4 finding Hole on Ladder.', duration=5.0, bubbles=False)
          #raw_input('Hit Enter to fine tune')
          #start_talking_time = time.time()
          hole_detection_one_acc += distributed_cmd('search_ladder_fine', robot['fine_perception'], ack_fine, 'Hole found.  Fine Tuning.', duration=2.0, bubbles=False)
          #end_talking_time = time.time()
          #diff = end_talking_time - start_talking_time
          #if diff < 8.0:
          #  rospy.sleep(diff)
      
      with open("/home/drl-mocap/time.txt",'a') as ftime:
        ftime.write('Ladder hole search %f SECS\n'%(time.time()-lasttime))
        lasttime = time.time()

      if usekinect:
          centralized_cmd('stop_tracking_ladder', robot['master'], 'Robots 1 and 3, pick up the panel.')

      # Step 1) Go to rack.

      distributed_base_cmd('go_to_rack', 0.0, pregrasp_radius, duration=5.0)
      
      # Step 2) Grasp the wing skin.
      #distributed_cmd('grasp_wing_side_pose', pause=True)
      if voice:
          wait_for('grab the panel')
          text_to_speech['master'].publish('Grabbing the panel.')
      distributed_cmd('grasp_wing_side_pose', pause=True, duration=2.0)
      
      #distributed_base_cmd('arm_feedback_to_grasp_wing', 0.0, grasp_radius)

      distributed_base_cmd('arm_feedback_to_grasp_wing_side', 0.0, grasp_radius_intermediate, duration=1.0)

      distributed_base_cmd('arm_feedback_to_grasp_wing_side', 0.0, grasp_radius, duration = 1.0)
      #raw_input('hit enter')

      centralized_cmd('establish_fleet_formation', duration=1.0)

      rospy.sleep(1.0) #TODO: turn this into a try loop
      centralized_cmd('nudge_skin', duration=2.0)

      centralized_cmd('pause_fleet')
      distributed_cmd('close_gripper', speech='Robots 1 and 3 have the panel.', duration=1.0)
      centralized_cmd('resume_fleet')
      rospy.sleep(0.2)
      #raw_input('closed')
      # Step 3) Establish a fleet formation.
      set_view('panel')
      centralized_cmd('tuck_arms', duration=2.0)
      start_tuck_arms = time.time()

      # Step 4) Lift the wing skin.
      centralized_cmd('lift_wing_skin', robot['carry'], duration=2.0)
      while (time.time()-start_tuck_arms) < 6.5:
          # busy waiting to make sure tucking ends.
          time.sleep(0.5)

      num_points = 1
      for i in range(num_points):
        angle_diff=(np.pi / 2 * (i + 1)/num_points)
        print i
        distributed_joint_cmd('set_joint_5', angle=angle_diff)

      #centralized_cmd('lift_wing_skin', 'Transport the wing skin.')

      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)

      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      #centralized_cmd('fix_grasp')
      #centralized_cmd('nudge_skin')

      #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)  

      # Step 5) Transport the wing skin.  Must avoid obstacles.
      centralized_cmd('rotate_to_face_ladder', robot['master'], 'Robots 1 and 3, move the panel to the ladder.', duration=5.0)


      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0, duration=1.0)
      #distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1, duration=1.0)

      #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0, duration=1.0)
      #distributed_cmd('close_gripper',[robot['carry'][1]],ack_carry1, duration=1.0)        

      with open("/home/drl-mocap/time.txt",'a') as ftime:
        ftime.write('grasp and lift skin %f SECS\n'%(time.time()-lasttime))
        lasttime = time.time()

      centralized_cmd('translate_to_ladder', duration=10.0)
      
      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)

      if new_2nd_hole_alignment:
        #skewed = 2.0
        skewed = 6.0
      centralized_cmd('rotate_into_ladder_alignment', robot['master'],  'Robots 1 and 3, align the panel to ladder.')
      if usekinect:
          centralized_cmd('start_tracking_skin')
          time.sleep(2.0)
          centralized_cmd('stop_tracking_ladder') # there is a bug we need to call again
          time.sleep(2.0)
          centralized_cmd('start_tracking_skin') # there is a bug we need to call again
      
      # Step 6) Position the wing skin over the wing ladder.
      #TODO: uncomment out when camera robot is needed
      #centralized_cmd('move_to_camera_pose', 'Activate camera for precise alignment')
      #centralized_cmd('start_visual_tracking', pause=True)
      #centralized_cmd('visual_servo_alignment')
      #centralized_cmd('stop_visual_tracking')
      
      #raw_input("Hit Enter to lower wing skin.\n")
      # Step 7) Lower the wing skin onto the ladder.

      # open close grippers quickly
      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1)
      #distributed_cmd('close_gripper',[robot['carry'][1]],ack_carry1)

      

      #with open("/home/drl-mocap/time.txt",'a') as ftime:
      #  ftime.write('skin-ladder alignment %f SECS\n'%(time.time()-lasttime))
      #  lasttime = time.time()

      set_view('default')
      if usehokuyo:
          #raw_input("Hit Enter to search skin hole.\n")
          while not rospy.is_shutdown():
              errors += 1
              
              #centralized_cmd('lower_wing_skin', robot['coarse_perception'], 'Tracking position of panel.')
              centralized_cmd('translate_to_ladder')
              centralized_cmd('rotate_into_ladder_alignment', speech='Robots 1 and 3 macro-aligning the panel')

              distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)-0.05))
              if usekinect and reallyusekinect:
                  #centralized_cmd('lower_wing_skin', robot['coarse_perception'], 'Tracking position of panel.')
                  centralized_cmd('lower_wing_skin', ['drc2'], 'Tracking position of panel.')

              #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
              #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)
              hole_detection_carry_acc += distributed_cmd('search_skin', robot['fine_perception'], ack_fine)
              resp = check_success[robot['fine_perception'][0]]()
              if not resp.success:
                  print 'NO SUCCESS after search_skin'
                  continue
              hole_detection_carry_acc += centralized_cmd('lower_wing_skin_1', robot['carry'], 'Robots 1 and 3, micro-aligning panel to ladder.')
              #distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)-0.05))
              hole_detection_carry_acc += distributed_cmd('prealign_holes', robot['fine_perception'], ack_fine)
              resp = check_success[robot['fine_perception'][0]]()
              if not resp.success:
                  print 'NO SUCCESS after prealign_holes'
                  distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)))
                  continue
              hole_detection_carry_acc += distributed_cmd('align_holes', robot['fine_perception'], ack_fine)
              resp = check_success[robot['fine_perception'][0]]()
              if not resp.success:
                  print 'NO SUCCESS after align_holes'
                  distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)))
                  continue
              hole_detection_carry_acc += distributed_cmd('prealign_holes2', robot['fine_perception'], ack_fine)
              resp = check_success[robot['fine_perception'][0]]()
              if not resp.success:
                  print 'NO SUCCESS after prealign_holes2'
                  distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)))
                  continue
              hole_detection_carry_acc += distributed_cmd('align_holes2', robot['fine_perception'], ack_fine)
              resp = check_success[robot['fine_perception'][0]]()
              if not resp.success:
                  print 'NO SUCCESS after align_holes2'
                  distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)))
                  continue
              else:
                  break
          
          #raw_input("Hit Enter to align holes.\n")
          
          #distributed_cmd('align_holes', robot['fine_perception'], ack_fine)
          
          #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
          #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)
          #distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1)
          #distributed_cmd('close_gripper',[robot['carry'][1]],ack_carry1)

          #raw_input("Hit Enter to lower wing skin final.\n")
          #centralized_cmd('lower_wing_skin_1', robot['carry'], 'Aligning panel to ladder.')
          #distributed_joint_cmd('set_joint_5', angle=((np.pi/2.0)-0.05))
          
          # open close grippers quickly
          #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
          #distributed_cmd('close_gripper',[robot['carry'][0]],ack_carry0)
          #distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1)
          #distributed_cmd('close_gripper',[robot['carry'][1]],ack_carry1)

          #raw_input("Hit Enter to align holes.\n")
      
      #centralized_cmd('lower_wing_skin_2', robot['carry'])

      
      #raw_input("Hit Enter to finalize.\n")
      # Step 8) Break the fleet formation.
      
       # Step 9) Release the wing skin.
      
      centralized_cmd('lower_wing_skin_2', robot['carry'])
      hole_detection_carry_acc += centralized_cmd('pause_fleet',robot['fine_perception'],  'Robot 4 can confirm the panel is in Position.  Ready to insert fastener.')

      #print_ladder_skin_offset()

      distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, -np.pi/2.0, 'hole_1', 'Inserting fastener.')

      centralized_cmd('resume_fleet',duration=1.0)
      centralized_cmd('lower_wing_skin_3', robot['carry'])

      distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_1', "Fastener number 1 installed.", bubbles=False)

      # if usevisinsp:
      # raw_input("Hit enter to run visual inspection.\n")
      # while not run_visinsp('fastner1') and not rospy.is_shutdown():
      while not rospy.is_shutdown():
        if usevisinsp:
          text_to_speech['master'].publish('Verifying fastener number 1 attachment.')
          #raw_input("Hit enter to run visual inspection.\n")
          if run_visinsp('fastner1'):
            text_to_speech['master'].publish('Fastener number 1 attached successfully.')
            break
          else:
            text_to_speech['master'].publish('Fastener number 1 attachment failed. Retrying insertion.')
        else:
          break
        distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, -np.pi/2.0, 'hole_1', 'Inserting fastener.')
        distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_1', "Fastener number 1 installed.", bubbles=False)

      #if usekinect:
      #  centralized_cmd('move_camera_robot_away', 'move camera robot away')



      """
      if usekinect:
        distributed_cmd('back_away', robot['coarse_perception'], ack_coarse, 'Robot 2 is getting out of the way')
        for i in range(5):
            distributed_cmd('back_away', robot['coarse_perception'], ack_coarse)
      """
  
 
  #############BEGIN HOLE 2 ##################
  
  if new_2nd_hole_alignment:
      #centralized_cmd('pause_fleet', robot['fine_perception'])
      centralized_cmd('break_fleet_formation', robot['fine_perception'])
      distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      distributed_cmd('back_away',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('back_away',[robot['carry'][0]],ack_carry0)
      #centralized_cmd('resume_fleet',duration=1.0)
      #skewed = 3.0
      #centralized_cmd('rotate_into_ladder_alignment', speech='Robots 1 and 3 macro-aligning the panel')
      vicon_acc +=  distributed_cmd('align_with_hole_2', robot['fine_perception'], ack_fine, 'Robot 4 is checking hole position.', bubbles=False)
      if usehokuyo:
          hole_detection_other_acc += distributed_cmd('search_ladder_2_bottom', robot['fine_perception'], ack_fine, bubbles=False)
          hole_detection_other_acc += distributed_cmd('search_ladder_fine_2_bottom', robot['fine_perception'], ack_fine, bubbles=False)
      #skewed = -23.0
      #centralized_cmd('rotate_into_ladder_alignment', speech='Robots 1 and 3 macro-aligning the panel')
      distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1)
      distributed_cmd('circular_motion',[robot['carry'][1]],ack_carry1)

      distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, -np.pi/2.0, 'hole_2', 'Panel aligned to second hole.  Inserting fastener.', bubbles=False)
      distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_2', 'Fastener inserted.  Proceeding to hole 3.', bubbles=False)

      while not rospy.is_shutdown():
        if usevisinsp:
          #raw_input("Hit enter to run visual inspection.\n")
          text_to_speech['master'].publish('Verifying fastener number 2 attachment.')
          if run_visinsp('fastner3'): # fastner3 because we changed the order of insertions.
            text_to_speech['master'].publish('Fastener number 2 attached successfully.')
            break
          else:
            text_to_speech['master'].publish('Fastener number 2 attachment failed. Retrying insertion.')
        else:
          break
        distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, -np.pi/2.0, 'hole_2', 'Panel aligned to second hole.  Inserting fastener.', bubbles=False)
        distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_2', 'Fastener inserted.  Proceeding to hole 3.', bubbles=False)


      #centralized_cmd('break_fleet_formation', robot['fine_perception'])
      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('back_away',[robot['carry'][0]],ack_carry0)
      #distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)

      #distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1)
      distributed_cmd('back_away',[robot['carry'][1]],ack_carry1)

      #Back off
      distributed_base_cmd('go_to_rack', 0.0, pregrasp_radius, robot['carry'], ack_carry,'Robots 1 and 3 are getting out of the way')
  else:

      centralized_cmd('break_fleet_formation', robot['fine_perception'])
      distributed_cmd('open_gripper',[robot['carry'][0]],ack_carry0)
      distributed_cmd('back_away',[robot['carry'][0]],ack_carry0)
      distributed_cmd('open_gripper',[robot['carry'][1]],ack_carry1)
      distributed_cmd('back_away',[robot['carry'][1]],ack_carry1)

      distributed_cmd('push_pose', robot['pushing_robot'], ack_push, speech='Robot 1 is aligning panel to second hole')

      #Go to one side
      distributed_base_cmd('go_to_ladder', -np.pi/2.0, prepush_radius + 0.15, robot['pushing_robot'], ack_push)
      #push
      distributed_cmd('push_first', robot['pushing_robot'], ack_push, speech='Robot 1 is aligning the first side')
      # Back up
      distributed_cmd('push_first_backup', robot['pushing_robot'], ack_push)
      # Move arm up
      distributed_cmd('arm_up_pose', robot['pushing_robot'], ack_push)
      
      #Go to other side
      distributed_base_cmd('go_to_ladder', -5 * np.pi/4, prepush_radius + 0.15, robot['pushing_robot'], ack_push, speech='Robot 1 is aligning second side')
      distributed_cmd('push_pose', robot['pushing_robot'], ack_push, speech='Robot 1 is aligning panel to second hole')
      #Push
      distributed_cmd('push_second', robot['pushing_robot'], ack_push)
      # Backup
      distributed_cmd('push_second_backup', robot['pushing_robot'], ack_push)
      # Move arm up
      distributed_cmd('arm_up_pose', robot['pushing_robot'], ack_push)

      #Back off
      distributed_base_cmd('go_to_rack', 0.0, pregrasp_radius, robot['carry'], ack_carry,'Robots 1 and 3 are getting out of the way')

      if usekinect and usevisinsp:    
        centralized_cmd('move_camera_robot_to_inspect_ladder', 'move camera robot to inspect ladder')
        
      #perception gets close
      distributed_base_cmd('go_to_ladder', np.pi/2.0, pregrasp_radius, robot['fine_perception'], ack_fine, bubbles=False)

      while not rospy.is_shutdown():
        vicon_acc +=  distributed_cmd('align_with_hole_2', robot['fine_perception'], ack_fine, 'Robot 4 is checking hole position.', bubbles=False)

        if usehokuyo:
          hole_detection_other_acc += distributed_cmd('search_ladder_2', robot['fine_perception'], ack_fine, bubbles=False)
          hole_detection_other_acc += distributed_cmd('search_ladder_fine_2', robot['fine_perception'], ack_fine, bubbles=False)

        distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, -np.pi/2.0, 'hole_2', 'Panel aligned to second hole.  Inserting fastener.', bubbles=False)

        #distributed_cmd('LiftTool', robot['fine_perception'], ack_fine, 'Fastener inserted.  Proceeding to hole 3.')
        distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_2', 'Fastener inserted.  Proceeding to hole 3.', bubbles=False)

        if usevisinsp:
          #raw_input("Hit enter to run visual inspection.\n")
          text_to_speech['master'].publish('Verifying fastener number 2 attachment.')
          if run_visinsp('fastner3'):
            text_to_speech['master'].publish('Fastener number 2 attached successfully.')
            break
          else:
            text_to_speech['master'].publish('Fastener number 2 attachment failed. Retrying insertion.')
        else:
          break

  #subprocess.call(['rosnode', 'kill', '/drc2/youbot_oodl_driver'])
  #rospy.sleep(10.0)

  ###############BEGIN HOLE 3####################

  if usekinect and usevisinsp:
    centralized_cmd('move_camera_robot_away', 'move camera robot away')

  distributed_cmd('back_away',robot['fine_perception'],ack_fine)
  distributed_cmd('back_away',robot['fine_perception'],ack_fine)
  distributed_base_cmd('go_to_ladder', -np.pi/2.0, prepush_radius, robot['fine_perception'], ack_fine, bubbles=False)
  
  if usekinect and usevisinsp:
    centralized_cmd('move_camera_robot_to_inspect_ladder', 'move camera robot to inspect ladder')
    
  while not rospy.is_shutdown():
    vicon_acc +=  distributed_cmd('align_with_hole_3', robot['fine_perception'], ack_fine, 'Robot 4 is checking hole position.', bubbles=False)

    if usehokuyo:
      hole_detection_other_acc += distributed_cmd('search_ladder_3', robot['fine_perception'], ack_fine, bubbles=False)
      hole_detection_other_acc += distributed_cmd('search_ladder_fine_3', robot['fine_perception'], ack_fine, bubbles=False)

    distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, np.pi/2.0, 'hole_3', 'Hole Aligned.  Inserting Fastener.', bubbles=False)

    #distributed_cmd('LiftTool', robot['fine_perception'], ack_fine, fastener inserted.  Proceeding to hole 4.)
    distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_3',  'Fastener inserted.  Proceeding to hole 4.', bubbles=False)

    if usevisinsp:
      #raw_input("Hit enter to run visual inspection.\n")
      text_to_speech['master'].publish('Verifying fastener number 3 attachment.')
      if run_visinsp('fastner2'):
        text_to_speech['master'].publish('Fastener number 3 attached successfully.')
        break
      else:
        text_to_speech['master'].publish('Fastener number 3 attachment failed. Retrying insertion.')
    else:
      break
  
  ###################BEGIN HOLE 4 ################

  distributed_base_cmd('go_to_ladder', -np.pi/2.0, prepush_radius, robot['fine_perception'], ack_fine, bubbles=False)

  while not rospy.is_shutdown():
    vicon_acc +=  distributed_cmd('align_with_hole_4', robot['fine_perception'], ack_fine, 'Robot 4 is checking hole position.', bubbles=False)

    if usehokuyo:
      hole_detection_other_acc += distributed_cmd('search_ladder_4', robot['fine_perception'], ack_fine, bubbles=False)
      hole_detection_other_acc += distributed_cmd('search_ladder_fine_4', robot['fine_perception'], ack_fine, bubbles=False)

    distributed_insert_cmd('Insert', robot['fine_perception'], ack_fine, np.pi/2.0, 'hole_4', 'Hole Aligned.  Inserting Fastener.', bubbles=False)

  #distributed_cmd('LiftTool', robot['fine_perception'], ack_fine, "Fastener is inserted.  Job's finished.")
    #distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_4', "Fastener is inserted.  Job is finished.", bubbles=False)
    distributed_insert_cmd('LiftTool', robot['fine_perception'], ack_fine, 0., 'hole_4', "Fastener is inserted.  ", bubbles=False)

    if usevisinsp:
      #raw_input("Hit enter to run visual inspection.\n")
      text_to_speech['master'].publish('Verifying fastener number 4 attachment.')
      if run_visinsp('fastner4'):
        text_to_speech['master'].publish('Fastener number 4 attached successfully.')
        break
      else:
        text_to_speech['master'].publish('Fastener number 4 attachment failed. Retrying insertion.')
    else:
      break

finally:
  with open("/home/drl-mocap/time.txt",'a') as ftime:
    ftime.write('hole_detection_one %f SECS\n' % hole_detection_one_acc)
    ftime.write('carry %f SECS\n' % hole_detection_carry_acc)     
    ftime.write('hole_detection_other %f SECS\n' % hole_detection_other_acc)
    ftime.write('vicon %f SECS\n' % vicon_acc)
    ftime.write('errors %f \n' % errors)
    ftime.write('TOTAL %f SECS\n' % (starttime - time.time()))

