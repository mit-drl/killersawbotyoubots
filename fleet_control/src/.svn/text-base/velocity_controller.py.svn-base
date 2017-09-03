#!/usr/bin/env python
# -*- coding: utf-8 -*-


#TODO: prune this for what we're not using
import roslib; roslib.load_manifest('fleet_control')
import rospy
import numpy as np
import openravepy as orpy
import re
import time
from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import youbotik as yi

import threading
import sys
import tf
import tf.transformations as tr
from mit_msgs.msg import MocapPosition

from straight_path import *


from assembly_common.srv import BasePose

listener = tf.TransformListener()

class VelocityController:
    def __init__(self, pub):
        np.set_printoptions(suppress=True)
        np.set_printoptions(precision=4)

        self.pub = pub
        self.env = orpy.Environment() # create openrave environment
        #TODO: make this openrave environment the same as the one in the other class
        self.env.SetViewer('qtcoin') # attach viewer (optional)
        self.env.Load(path + '/robots/kuka-youbot.robot.xml') 
        self.robot = self.env.GetRobots()[0]
        print 'robot is'
        print self.robot
        self.manip = self.robot.GetManipulators()[0]
        print 'manip is'
        print self.manip
                
        self.cur_pos = np.zeros((4, 4))
        self.sem = threading.Semaphore()
        print 'finished constructor'
        
        
    def create_trajectory(self, goal):
        #We should know our own position, so we don't need the start point
        #The goal is where we want to get to in R6
        #The velocity is a parameter specifying how fast we want to go.
        global listener
        (trans,quat) = listener.lookupTransform('/map', my_name + '_arm', rospy.Time(0))
        euler = tr.euler_from_quaternion(quat)
        
        start = Twist()
        start.linear.x = trans[0]
        start.linear.y = trans[1]
        start.linear.z = trans[2]
        start.angular.x = euler[0]
        start.angular.y = euler[1]
        start.angular.z = euler[2]
        
        self.path = StraightPath(start, goal)
        
    def initialize_trajectory(self, speed):
        self.path.initialize_list(speed)
        
    def execute_list(self, baseless):
        while True:
            if baseless:
                ret_val = path.pop_and_call(self.call_func_baseless)
            else:
                ret_val = path.pop_and_call(self.call_func_base)
            if ret_val == None:
                break #Then we're done
                
    def call_func_baseless(self, arg):
        pub_baseless.publish(arg)
        
    def call_func_base(self, arg):
        pub_base.publish(arg)
           
        
        




path = ''


if len(sys.argv) < 2:
    print "Usage: youbot_ik.py <model_location>"
else:
    path = sys.argv[1]
rospy.init_node('velocity_controller')
print 'init node'
my_name = rospy.get_namespace()
#TODO: input topic?


out_command_topic_baseless = my_name + '/ik_arm_command'
out_command_topic_base = my_name + '/ik_command_fine'


pub_baseless = rospy.Publisher(out_command_topic_baseless, Twist)
pub_base = rospy.Publisher(out_command_topic_base, Twist)


print 'inited publisher'
vc = VelocityController(pub)
print 'inited constructor'
joint_states_topic = my_name + '/joint_states'
rospy.Subscriber(in_command_topic_baseless, Twist, ik.handle_command_baseless) #TODO: change name
rospy.Subscriber(in_command_topic_base, Twist, ik.handle_command_base) #TODO: change name
rospy.Subscriber(joint_states_topic,  JointPositions, ik.updatePositions) 
print 'done initializing!'
rospy.spin()
