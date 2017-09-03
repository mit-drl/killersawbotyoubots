#!/usr/bin/env python

import roslib; roslib.load_manifest('fleet_control')

from brics_actuator.msg import JointPositions,JointValue
from geometry_msgs.msg import Twist,PoseStamped
from fleet_control.msg import Fleet
from sensor_msgs.msg import Joy
import threading
import rospy
import math
import sys


# Joypad button controls
BUTTON0 = 0
GRIPPER_CLOSE_BUTTON = 1
BUTTON2 = 2
GRIPPER_OPEN_BUTTON = 3
BUTTON4 = 4
BUTTON5 = 5
BUTTON6 = 6
BUTTON7 = 7
SPEED_DOWN_BUTTON = 8
STOP_BUTTON = 9
SPEED_UP_BUTTON = 10

# Joypad axis controls
BASE_Y_AXIS = 0
BASE_X_AXIS = 1
BASE_ROT_LEFT_AXIS = 2
AXIS3 = 3
AXIS4 = 4
BASE_ROT_RIGHT_AXIS = 5
AXIS6 = 6
ARM_Z_AXIS = 7

ROT_SCALE = 0.5
DEFAULT_VEL = 0.4
STEP_VEL = 0.1
MAX_VEL = 0.7
MAX_VEL_Z = 0.02

CMD_TIMEOUT = 0.25 # seconds

default_axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]
default_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

gripper_pub = dict()
finger_names = [ "gripper_finger_joint_l", "gripper_finger_joint_r" ]
finger_min = 0.0
finger_max = 0.011499

class ThreadClass(threading.Thread):
    def __init__(self):
        super(ThreadClass, self).__init__()
        self.axes = [ vv for vv in default_axes ]
        self.buttons = [ vv for vv in default_buttons ]
        self.axis_valid = [ False for ii in range(8) ]

    def run(self):
        global lock
        global cond

        lock = threading.Lock()
        cond = threading.Condition(lock)

        sent_zeros = False
        last_gripper_cmd_was_open = False

        while not rospy.is_shutdown():
            lock.acquire()
            #print "LOCK acquired by thread"
            cond.wait(CMD_TIMEOUT)

            # Here if either timed out or got a button press
            xx = MAX_VEL   * self.axes[BASE_X_AXIS]
            yy = MAX_VEL   * self.axes[BASE_Y_AXIS]
            zz = MAX_VEL_Z * self.axes[ARM_Z_AXIS]

            if not self.axis_valid[BASE_ROT_LEFT_AXIS]:
                self.axes[BASE_ROT_LEFT_AXIS] = default_axes[BASE_ROT_LEFT_AXIS]
            if not self.axis_valid[BASE_ROT_RIGHT_AXIS]:
                self.axes[BASE_ROT_RIGHT_AXIS] = default_axes[BASE_ROT_RIGHT_AXIS]

            all_default = not [i for i, j in zip(self.buttons, default_buttons)\
                          if i != j] and not [i for i, j in zip(self.axes, \
                          default_axes) if i != j]

            #print "all_default: %s   sent_zeros: %s" % (all_default, sent_zeros)

            if all_default:
                if sent_zeros:
                    lock.release()
                    #print "LOCK released by thread"
                    continue
                sent_zeros = True
            else:
                sent_zeros = False

            left = self.axes[BASE_ROT_LEFT_AXIS]
            right = self.axes[BASE_ROT_RIGHT_AXIS]

            if left < 0.99 and right < 0.99:
                rot = 0.0
            elif left >= 0.99:
                rot = ROT_SCALE * (right - 1.0)
            else:
                rot = ROT_SCALE * (1.0 - left)

            #print "left: %f   right: %f   rot: %f" % (left, right, rot)

            if xx == 0.0 and yy == 0.0:
		xx = self.axes[AXIS4] * vel
                yy = self.axes[AXIS3] * vel

            lock.release()
            #print "LOCK released by thread"

            twist = Twist()
            twist.linear.x = xx
            twist.linear.y = yy
            twist.linear.z = zz
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = rot

            #print "SEND: %s" % twist
            cmd_vel.publish(twist)

            if (self.buttons[GRIPPER_OPEN_BUTTON] and not last_gripper_cmd_was_open) or (self.buttons[GRIPPER_CLOSE_BUTTON] and last_gripper_cmd_was_open):
                # Send a gripper command
                last_gripper_cmd_was_open = self.buttons[GRIPPER_OPEN_BUTTON]
                val = finger_min if self.buttons[GRIPPER_CLOSE_BUTTON] else finger_max

                grip = JointPositions()
                for ii in range(2):
                    jv = JointValue()
                    jv.timeStamp = rospy.Time.now()
                    jv.joint_uri = finger_names[ii]
                    jv.unit = 'm'
                    jv.value = val
                    grip.positions.append(jv)
                for rr in robot_names:
                    gripper_pub[rr].publish(grip)


def handle_joy(msg):
    global lock, cond

    send_cmd = False

    lock.acquire()
    #print "LOCK acquired by callback"

    if len(msg.buttons) != len(th.buttons) or len(msg.axes) != len(th.axes):
        print "%d != %d   or   %d != %d" % (len(msg.buttons), len(th.buttons), len(msg.axes), len(th.axes))
        send_cmd = True
    send_cmd |= bool([i for i, j in zip(msg.buttons, th.buttons) if i != j])
    send_cmd |= bool([i for i, j in zip(msg.axes, th.axes) if i != j])

    for ii in range(len(msg.axes)):
        if th.axes[ii] != msg.axes[ii] and msg.axes[ii] != 0.0:
            th.axis_valid[ii] = True

    if send_cmd:
        for ii in range(len(msg.buttons)):
            th.buttons[ii] = msg.buttons[ii]
        for ii in range(len(msg.axes)):
            th.axes[ii] = msg.axes[ii]
        cond.notify()
    lock.release()
    #print "LOCK released by callback"


def wait_for_subscriber(pub):
    pass


if __name__ == "__main__":
    rospy.init_node('multi_robot_joy')
    argv = rospy.myargv(argv=sys.argv)

    vel = DEFAULT_VEL

    if len(argv) < 2:
        print "USAGE: %s list of robot names" % argv[0]
        sys.exit(1)

    robot_names = []

    for ii in range(1,len(argv)):
        robot_names.append(argv[ii])

    for rr in robot_names:
        gripper_pub[rr] = rospy.Publisher('/' + rr + '/arm_1/gripper_controller/position_command', JointPositions)

    print robot_names
    cmd_vel = rospy.Publisher('/multi_cmd', Twist, latch=True)
    origin_cmd = rospy.Publisher('/transform_origin', PoseStamped)
    set_fleet = rospy.Publisher('/set_fleet', Fleet, latch=True)

    th = ThreadClass()
    th.start()

    joy_sub = rospy.Subscriber('/joy', Joy, handle_joy)

    fleet = Fleet()
    fleet.group.extend(robot_names)
    wait_for_subscriber(set_fleet)
    set_fleet.publish(fleet)

    rospy.spin()
