#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('fleet_control')
import rospy
import numpy as np
from path import Path
from geometry_msgs.msg import Twist, Vector3



class StraightPath(Path):

    def __init__(self, start, end):
        #start and finish are 6-dof coordinates
        #should be a list of Twists
        Path.__init__(self, start, end)
        
    def initialize_list(self, speed):
        #the movement is just going to be 3-dof
        sx = self.start.linear.x
        sy = self.start.linear.y
        sz = self.start.linear.z
        ex = self.end.linear.x
        ey = self.end.linear.y
        ez = self.end.linear.z
        dx = ex - sx
        dy = ey - sy
        dz = ez - sz
        dist = np.sqrt(dx*dx + dy*dy + dz*dz)
        
        num_points = int( dist / speed )
        
        
        for i in range(num_points + 1):
            twist = Twist()
            twist.linear = Vector3()
            
            #let's just set it right to the endpose though.
            twist.angular = self.end.angular
            
            #interpolate
            twist.linear.x = sx + (ex - sx) * i / num_points
            twist.linear.y = sy + (ey - sy) * i / num_points
            twist.linear.z = sz + (ez - sz) * i / num_points
        
            self.points.append(twist)
        
 
    def pop_and_call(self, func):
        return Path.pop_and_call(func)

start = Twist()
start.linear = Vector3()
start.linear.x = 1.0
start.linear.y = 1.0
start.linear.z = 1.0
start.angular = Vector3()

end = Twist()
end.linear = Vector3()
end.linear.x = 3.0
end.linear.y = 3.0
end.linear.z = 5.0
end.angular = Vector3()
end.angular.x = 1.0

sp = StraightPath(start, end)
sp.initialize_list(1)

for elem in sp.points:
    print elem
