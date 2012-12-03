#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_autonomy')
import rospy

from sensor_msgs.msg import Image
import cv_bridge as bridge


def got_image(msg):
    global target_pub

    print "in got_msg"

    cv_image = bridge.imgmsg_to_cv(msg)

if __name__ == "__main__":
    
    rospy.init_node("opencv")

    #try:
    #    import psyco
    #    psyco.full()
    #except IOError:
    #    pass

    rospy.Subscriber("/ardrone/front/image_raw", Image, got_image)
