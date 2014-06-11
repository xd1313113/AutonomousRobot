#!/usr/bin/env python  
# Adds a new frame as a parent of odom. This makes the location of the
# robot at the beginning of the program be the center of the world map
import roslib
roslib.load_manifest('project')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('p3dx_new_odom_broadcaster')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(20.0)
    try:
        listener.waitForTransform("base_link", "odom", rospy.Time(), rospy.Duration(5))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    add_frame = False
    try:
        (trans,rot) = listener.lookupTransform("base_link", "odom", rospy.Time())
        add_frame = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerror("Error: cannot transform between odom and base_link")

    #print "Got: ", trans, rot
    while not rospy.is_shutdown() and add_frame:
        br.sendTransform(trans,
                         rot,
                         rospy.Time.now(),
                         "/odom",
                         "/map",
                         )

        rate.sleep()
