#!/usr/bin/env python
import roslib; roslib.load_manifest('project')
import rospy
import sys
from p2os_driver.msg import MotorState

if __name__ == '__main__':
    rospy.init_node('p3dx_robot_motor', anonymous=True)
    pub = rospy.Publisher('/cmd_motor_state', MotorState)
    args = rospy.myargv(argv=sys.argv)
    if args[1].strip() == 'on':
        print "Publishing 1"
        pub.publish(MotorState(1))
    elif args[1].strip() =='off':
        pub.publish(MotorState(0))
    else:
        print "Usage: robot_motor.py [on | off]"
