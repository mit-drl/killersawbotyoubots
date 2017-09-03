#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('fleet_control')
import rospy
import numpy as np
import openravepy as orpy
import re
import time
import euclid
import copy
import youbotpy
from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Pose
from fleet_control.msg import VelTwist, Fleet
from fleet_control.srv import Switch

import IPython
import threading
import sys
import tf
import tf.transformations as tr
from mit_msgs.msg import MocapPosition
from std_msgs.msg import Bool

from vicon_utils import *

temp = None

ang_thresh = 0.2
#ang_speed = 0.05
simulation = False
robot = ['drc1', 'drc3'] #TODO: un-hardcode this
control = 'drc1'

fleet_origin = None

def get_fleet_origin_in_frame(frame):
    global_fleet_pose = PoseStamped()
    global_fleet_pose.header.frame_id = '/map'
    global_fleet_pose.pose = fleet_origin
    return transform_by_subjects(global_fleet_pose, frame)


def from_frame_to_fleet_origin_frame(frame, twist):
    twist = copy.copy(twist)
    local_fleet_pose = get_fleet_origin_in_frame(frame)
    qt = euclid.Quaternion(local_fleet_pose.pose.orientation.w,
                           local_fleet_pose.pose.orientation.x,
                           local_fleet_pose.pose.orientation.y,
                           local_fleet_pose.pose.orientation.z)
    fleet_pitch, fleet_yaw, fleet_roll = qt.get_euler()
    # First represent the twist relative to fleet_frame, but represented in frame
    angular_vel_vector = np.array([0.0,0.0,twist.angular.z])
    frame_to_fleet_origin_vector = np.array([local_fleet_pose.pose.position.x,local_fleet_pose.pose.position.y,0.0])
    linear_vel_at_fleet_due_to_angular_vel_at_frame = np.cross(angular_vel_vector,frame_to_fleet_origin_vector)
    twist.linear.x = twist.linear.x + linear_vel_at_fleet_due_to_angular_vel_at_frame[0]
    twist.linear.y = twist.linear.y + linear_vel_at_fleet_due_to_angular_vel_at_frame[1]

    # Now represent everything in fleet_frame
    out = Twist()
    sy = np.sin(fleet_yaw)
    cy = np.cos(fleet_yaw)
    out.linear.x =  cy * twist.linear.x + sy * twist.linear.y
    out.linear.y = -sy * twist.linear.x + cy * twist.linear.y
    out.linear.z = twist.linear.z
    out.angular.z = twist.angular.z
    # Assume roll and pitch are zero
    return out


class TestSearch:

    #def establish_fleet_formation(self):
    #    set_fleet_topic = '/set_fleet'
    #    set_fleet = rospy.Publisher(set_fleet_topic, Fleet, latch=True)
    #    fleet = Fleet()
    #    fleet.group.extend(robot)
    #    set_fleet.publish(fleet)
    #    origin = PoseStamped()
    #    origin.header.frame_id = '/map'
    #    origin.pose = vicon_to_geometry_pose('drc4_arm')
    #    transform_origin_pub.publish(origin)

    def switch_handler(self,req):
        global control
        if req.skin is True:
            control = 'fleet'
            self.pub_base = rospy.Publisher('/multi_cmd', Twist, latch=True)
            return True
        else:
            control = 'drc1'
            self.pub_base = rospy.Publisher('/drc1/cmd_vel', Twist, latch=True)
            return False

    def set_robot_location(self):
        if simulation:
            position = self.robot.GetTransform()
            trans = np.array([position[0, 3], position[1, 3], position[2, 3]])
            euler = tr.euler_from_matrix(position)
            twist = Twist()
            twist.linear.x = trans[0]
            twist.linear.y = trans[1]
            twist.linear.z = trans[2]
            twist.angular.x = euler[0]
            twist.angular.y = euler[1]
            twist.angular.z = euler[2]
            
            pub_pos.publish(twist)
            #raw_input('published')
            
        else:
            global listener
            (trans,quat) = listener.lookupTransform('/map', my_name[:-1], rospy.Time(0))
            euler = tr.euler_from_quaternion(quat)

            cur_pos = tr.compose_matrix(angles=euler, translate=np.array(trans))
            with self.robot.GetEnv():
                self.robot.SetTransform(cur_pos)
            #TODO: is this topic really necessary?
            twist = Twist()
            twist.linear.x = trans[0]
            twist.linear.y = trans[1]
            twist.linear.z = trans[2]
            twist.angular.x = euler[0]
            twist.angular.y = euler[1]
            twist.angular.z = euler[2]
            pub_pos.publish(twist)
        #print "position is"
        #print twist
        
        
    def __init__(self, pub_base):
        self.pub_base = pub_base
        all_robot_names = ['drc1']
        envfile = "/home/drl-mocap/drl-youbot-openrave/models/environments/chair.env.xml"
        youbotenv = youbotpy.YoubotEnv(sim=True,viewer=True,env_xml=envfile, \
                               youbot_names=all_robot_names)
                               
        self.env = youbotenv.env
        youbots = youbotenv.youbots
        """
        for name in youbots:
            youbots[name].SetTransform(robot_start_poses[name])
            youbotenv.MoveGripper(name,0.01,0.01) # open grippers
        """
                                  
        #self.env = orpy.Environment() # create openrave environment
        if simulation:
            physics = orpy.RaveCreatePhysicsEngine(self.env, "ode")
            self.env.SetPhysicsEngine(physics)
        #self.env.SetViewer('qtcoin') # attach viewer (optional)
        #self.env.Load(path + '/robots/kuka-youbot.robot.xml') 
        self.robot = self.env.GetRobots()[0]
        self.manip = self.robot.GetManipulators()[0]
        with self.robot.GetEnv():
            self.robot.SetActiveManipulator(self.manip)
        self.cur_pos = np.zeros(5)
        self.lock = threading.Lock()
        self.robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
        self.planner = orpy.interfaces.BaseManipulation(self.robot,'BIRRT')
        
        print 'finished constructor'
    
    def update_vel(self, msg):
        with self.robot.GetEnv():
            self.robot.SetVelocity(np.array([msg.linear.x, msg.linear.y, msg.linear.z]), np.array([msg.angular.x, msg.angular.y, msg.angular.z]))

    def handle_base_command_fine(self, msg):
        #msg is a twist representing xyz euler angles
        try:
            self.lock.acquire()
            print 'got it'
            self.new_base_vel = msg.velocity.data
            self.new_dist_thresh = self.new_base_vel / 2.0
            self.new_x_goal = msg.target.linear.x
            self.new_y_goal = msg.target.linear.y
            self.new_yaw_goal = msg.target.angular.z
            self.new_target = True
        finally:
            self.lock.release()

    def run(self):            
        at_goal = True
        self.new_target = False
        traj = None
        while not rospy.is_shutdown():


            try:
                self.lock.acquire()
                if self.new_target:
                    print 'GOT NEW TARGET'
                    self.new_target = False
                    base_vel = self.new_base_vel
                    goal_config = np.zeros((3, 1))
                    goal_config[0] = self.new_x_goal
                    goal_config[1] = self.new_y_goal
                    goal_config[2] = self.new_yaw_goal
                    with self.robot.GetEnv():
                        traj = self.planner.MoveActiveJoints(goal=goal_config,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True)
                        waypoint_num = traj.GetNumWaypoints()
                        for i in range(waypoint_num):
                            wp = traj.GetWaypoint(i)
                            wp[3] *= base_vel
                            wp[4] *= base_vel
                            wp[5] *= base_vel
                  
            finally:
                self.lock.release()


            

            if traj is None:
                continue
                
            IPython.embed()
            self.robot.GetController().SetPath(traj)
            while not self.robot.GetController().IsDone():
                time.sleep(0.1)
                
            traj = None
            #TODO: set trajectory?
                    
                

def handle_fleet_origin(msg):
    global fleet_origin
    fleet_origin = msg

            
        
            
path = ''


if len(sys.argv) < 2:
    print "Usage: youbot_ik.py <model_location>"
else:
    path = sys.argv[1]
rospy.init_node('cmd_youbot_pos_trajectory')
listener = tf.TransformListener()
rospy.sleep(2.0) #Build the buffer

my_name = rospy.get_namespace()
if my_name is '/':
    my_name = '/drc1/'
    
print my_name

in_command_topic_base_fine = my_name + 'ik_command_fine'
out_command_topic_base = my_name + 'cmd_vel'
#out_command_topic_base = '/multi_cmd'
out_pos = my_name + 'location'
print 'publisher name is ', out_pos

pub_base = rospy.Publisher(out_command_topic_base, Twist, latch=True)

print out_pos
pub_pos = rospy.Publisher(out_pos, Twist, latch=True)

ts = TestSearch(pub_base)
th = threading.Thread(target=ts.run)
th.start()

rospy.Subscriber(in_command_topic_base_fine, VelTwist, ts.handle_base_command_fine,queue_size=1)
#rospy.Subscriber(out_command_topic_base, Twist, ts.update_vel)
switch = rospy.Service('/switch', Switch, ts.switch_handler) 
transform_origin_topic = '/transform_origin'

fleet_origin_sub = rospy.Subscriber('/fleet_origin', Pose, handle_fleet_origin)

#TODO: this won't be needed in the future as it's handled in the wing_transport_master
#transform_origin_pub = rospy.Publisher(transform_origin_topic, PoseStamped, latch=True)
#ts.establish_fleet_formation() #TODO: uncomment?

print 'done initializing!'

rospy.spin()

