#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('fleet_control')
import rospy
import numpy as np
from geometry_msgs.msg import Twist


class Path:

    def __init__(self, start, end):
        #start and finish are 6-dof coordinates
        if type(start) is not Twist or type(end) is not Twist:
            print "Please pass in Twists for the start and end coordinates"
            raise Exception
        self.start = start
        self.end = end
        self.points = []
        
    def initialize_list(self, speed):
        #For now do nothing, I guess.  Do stuff in the super-class
        pass
        
 
    def pop_and_call(self, func):
        if len(self.points.pop) > 0:
            return func(self.points.pop(0))
        else:
            return None
        
