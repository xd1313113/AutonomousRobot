#!/usr/bin/env python
import roslib; roslib.load_manifest('project')
import rospy
from geometry_msgs.msg import Twist, Vector3


def drive_back_and_forth():
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('p3dx_controller')
    m_forw=Twist(Vector3(1,0,0), Vector3(0,0,0))
    m_back=Twist(Vector3(-1,0,0), Vector3(0,0,0))
    while not rospy.is_shutdown():
        for i in range(20):
            pub.publish(m_forw)
            rospy.sleep(0.01)
        # Slight delay
        rospy.sleep(1.0)
        for i in range(20):
            pub.publish(m_back)
            rospy.sleep(0.01)
        # Slight delay
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        drive_back_and_forth()
    except rospy.ROSInterruptException:
        pass
