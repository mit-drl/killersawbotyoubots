#!/usr/bin/env python
# -*- coding: utf-8 -*-

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



from assembly_common.srv import BasePose



temp = None
listener = tf.TransformListener()

base_vel = 0.005
dist_thresh = 0.01
ang_thresh = 0.2
ang_speed = 0.05

class IkController:
    
    global temp
    def updatePositions(self, jp):
        self.sem.acquire()
        for ji in range(5):
            self.cur_pos[ji] = jp.position[ji]
            
        self.sem.release()
        

    def convert_to_np(self, vec):
        return np.array([vec.x, vec.y, vec.z])

    def set_robot_location(self):
        global listener
        
        
        print my_name
        (trans,quat) = listener.lookupTransform('/map', my_name, rospy.Time(0))
        euler = tr.euler_from_quaternion(quat)
        print trans
        print quat
        cur_pos = tr.compose_matrix(angles=euler, translate=np.array(trans))
        self.robot.SetTransform(cur_pos)
        

    def handle_command_baseless(self, msg):
        print 'got command'
        global temp

        #First thing's first: update robots
        
        self.set_robot_location()
        

        print 'finished upate'
    
        #TODO: servo fleet can just send the same goal and velocity over and over
    
     #TODO: convert this then to just go to that current step with small loop increments
     #increment sizes based off velocity
     #This doesn't loop - just makes one quick decision.
    
        target = tr.compose_matrix(angles=np.array([msg.angular.x, msg.angular.y, msg.angular.z]), translate=np.array([msg.linear.x, msg.linear.y, msg.linear.z]))
        
        #target = np.array([[  2.85857304e-02,  -4.36014737e-02,   9.98639962e-01, 6.05833276e-01],[  9.99591343e-01,   1.24685465e-03,  -2.85585263e-02,-1.73253385e-02],[  3.56102386e-08,   9.99048224e-01,   4.36193016e-02, 4.35463143e-01],[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,1.00000000e+00]])
        print tr.euler_from_matrix(target, 'rxyz')
        print target
        temp = orpy.misc.DrawAxes(self.env, target, 0.25, 2.0)
        self.sem.acquire()
        current = self.cur_pos.copy()
        self.sem.release()
        
        #Now that we have the current position and the target, let's do our inverse kinematics thing
        
        solutions = yi.FindIKSolutions(youbot=self.robot, eepose=target)
        if len(solutions) > 0:
            self.send_arm_msg(solutions[0])
        else:
            print 'no solutions!'
      
      
    def handle_command_base(self, msg):
        print 'got command'

        global temp

        #First thing's first: update robots
        
        self.set_robot_location()
    
    #TODO: servo fleet can just send the same goal and velocity over and over
    
     #TODO: convert this then to just go to that current step with small loop increments
     #increment sizes based off velocity
     #This doesn't loop - just makes one quick decision.
        target = tr.compose_matrix(angles=np.array([msg.angular.x, msg.angular.y, msg.angular.z]), translate=np.array([msg.linear.x, msg.linear.y, msg.linear.z]))
        #target = np.array([[  2.85857304e-02,  -4.36014737e-02,   9.98639962e-01, 6.05833276e-01],[  9.99591343e-01,   1.24685465e-03,  -2.85585263e-02,-1.73253385e-02],[  3.56102386e-08,   9.99048224e-01,   4.36193016e-02, 4.35463143e-01],[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,1.00000000e+00]])
        
        self.sem.acquire()
        current = self.cur_pos.copy()
        self.sem.release()
        
        #Now that we have the current position and the target, let's do our inverse kinematics thing
        
        base_solution, arm_solution = yi.FindIKAndBaseSolutions(youbot=self.robot, eepose=target, returnfirst=True)
        self.send_msg(arm_solution[0], base_solution[0])

    def handle_command_base_fine(self, msg):
            print 'got command'

            global temp

        #First thing's first: update robots
        
            self.set_robot_location()
        
        #TODO: servo fleet can just send the same goal and velocity over and over
        
         #TODO: convert this then to just go to that current step with small loop increments
         #increment sizes based off velocity
         #This doesn't loop - just makes one quick decision.
            target = tr.compose_matrix(angles=np.array([msg.angular.x, msg.angular.y, msg.angular.z]), translate=np.array([msg.linear.x, msg.linear.y, msg.linear.z]))
            #target = np.array([[  2.85857304e-02,  -4.36014737e-02,   9.98639962e-01, 6.05833276e-01],[  9.99591343e-01,   1.24685465e-03,  -2.85585263e-02,-1.73253385e-02],[  3.56102386e-08,   9.99048224e-01,   4.36193016e-02, 4.35463143e-01],[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,1.00000000e+00]])
            
            self.sem.acquire()
            current = self.cur_pos.copy()
            self.sem.release()
            
            #Now that we have the current position and the target, let's do our inverse kinematics thing
            
            base_solution, arm_solution = yi.FindIKAndBaseSolutions(youbot=self.robot, eepose=target, returnfirst=True)
            self.send_arm_msg(arm_solution[0])
            self.send_base_msg_fine(base_solution[0])


        
            
            
        
        
    def send_arm_msg(self, solution):
    

            
        # Find a solution to the target
        
        if solution is None:
            print "Can't move!"
            return
                
       
        
        #for q in path:
        #And finally, let's execute:
        print solution
        self.robot.SetDOFValues(solution, self.manip.GetArmIndices())
        positions = self.convert_to_real_youbot_joint_values(solution)
        jp = JointPositions()
        jp.positions = [JointValue() for i in range(5)]
        #print jp.positions
        now = rospy.get_rostime()

        for ji in range(5):
            jp.positions[ji].timeStamp = now
            jp.positions[ji].joint_uri = 'arm_joint_' + str(ji + 1)
            jp.positions[ji].unit = 'rad'
            jp.positions[ji].value = positions[ji]
            
            #print jp.positions[ji].value
        
        #print jp
        pub.publish(jp)

        
        time.sleep(0.2)

    def send_base_msg(self, base_solution):
        basepose = base_solution
        if basepose is not None:
            print basepose
            x = basepose[0,3]
            y = basepose[1,3]
            theta = np.arctan2(basepose[1,0],basepose[0,0])
		
            #TODO: is map right?
            #TODO: remove magic numbers
            base_command(x, y, theta, 0.05, 0.1, False, "/map")



    def send_base_msg_fine(self, base_solution):
        #TODO: remove magic numbers
        basepose = base_solution
        if basepose is not None:
            print basepose
            x = basepose[0,3]
            y = basepose[1,3]

            #First, find direction from current to target:
            cur = self.robot.GetTransform()
            print 'here we go'
            print cur
            tar = basepose
            print tar
            print np.linalg.inv(cur)
            transform = np.dot(tar, np.linalg.inv(cur))
            print transform

            x_goal = basepose[0, 3]
            y_goal = basepose[1, 3]

            

            while True:
                self.set_robot_location()
                (trans,quat) = listener.lookupTransform('/map', my_name, rospy.Time(0))
                print trans

                #linear:
                x_diff = x_goal - trans[0]
                y_diff = y_goal - trans[1]
                print x_diff
                print y_diff
                dist2 = np.sqrt(x_diff * x_diff + y_diff * y_diff)
                twist = Twist()
                twist.linear.x = -x_diff/dist2 * base_vel
                twist.linear.y = -y_diff/dist2 * base_vel
                twist.linear.z = trans[2]

                #angular:
                #TODO: figure it out
                """
                euler_goal = tr.euler_from_matrix(base_solution, axes='sxyz')
                euler_current = tr.euler_from_quaternion(quat)
                print euler_goal
                print euler_current 
                
                ang_diff_x = euler_goal[0] - euler_current[0]
                ang_diff_y = euler_goal[1] - euler_current[1]
                ang_diff_z = euler_goal[2] - euler_current[2]
                ang2 = np.sqrt(ang_diff_x * ang_diff_x + ang_diff_y * ang_diff_y + ang_diff_z * ang_diff_z)
                twist.angular.x = ang_diff_x/ ang2 * ang_speed
                twist.angular.y = ang_diff_y/ ang2 * ang_speed
                twist.angular.z = ang_diff_z/ ang2 * ang_speed
                """
                pub_base.publish(twist)
                print 'Distance is'
                print dist2
                print 'ang is'
                print ang2
                print 'twist is'
                print twist
                if dist2 < dist_thresh and ang2 < ang_thresh:
                    pub_base.publish(Twist())
                    break
                rospy.sleep(0.2)
                
                
		    
            
        
        
    
    def __init__(self, pub):
        np.set_printoptions(suppress=True)
        np.set_printoptions(precision=4)

        self.pub = pub
        self.env = orpy.Environment() # create openrave environment
        self.env.SetViewer('qtcoin') # attach viewer (optional)
        self.env.Load(path + '/robots/kuka-youbot.robot.xml') 
        self.robot = self.env.GetRobots()[0]
        print 'robot is'
        print self.robot
        self.manip = self.robot.GetManipulators()[0]
        print 'manip is'
        print self.manip
        
        with self.robot.GetEnv():
            self.robot.SetActiveManipulator(self.manip)
            yi.init(self.robot)
                
        self.cur_pos = np.zeros(5)
        self.sem = threading.Semaphore()
        print 'finished constructor'
                
        
        

    def convert_to_real_youbot_joint_values(self, q):
        jointdiff = np.array([2.949606435870417,
                                   1.1344640137963142,
                                  -2.548180707911721,
                                  1.7889624832941877,
                                  2.923426497090502])
        return jointdiff + q
        
        
        
path = ''


if len(sys.argv) < 2:
    print "Usage: youbot_ik.py <model_location>"
else:
    path = sys.argv[1]
rospy.init_node('ik')
print 'init node'
my_name = rospy.get_namespace()
if my_name is '/':
    my_name = '/drc1'
out_command_topic = my_name + '/arm_1/arm_controller/position_command'
out_command_topic_base = my_name + '/cmd_vel'
in_command_topic_baseless = my_name + '/ik_arm_command'
in_command_topic_base = my_name + '/ik_command'
in_command_topic_base_fine = my_name + '/ik_command_fine'

#TODO: figure out if these next two lines need reworking for namespaces and whatever?
base_service = my_name + '/robot_base_command'
base_command = rospy.ServiceProxy(base_service, BasePose)

print out_command_topic
print in_command_topic_baseless
pub = rospy.Publisher(out_command_topic, JointPositions)
print 'inited publisher'
ik = IkController(pub)
print 'inited constructor'
joint_states_topic = my_name + '/joint_states'
rospy.Subscriber(in_command_topic_baseless, Twist, ik.handle_command_baseless) 
rospy.Subscriber(in_command_topic_base, Twist, ik.handle_command_base)
rospy.Subscriber(in_command_topic_base_fine, Twist, ik.handle_command_base_fine)
rospy.Subscriber(joint_states_topic,  JointState, ik.updatePositions) 
pub_base = rospy.Publisher(out_command_topic_base, Twist)
print 'done initializing!'
rospy.spin()

