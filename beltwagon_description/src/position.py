#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest
import math
import actionlib
from collections import deque
from geometry_msgs.msg import Point, Twist
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from std_srvs.srv import *
import tf

#goal_list = []



#service callbacks
#def main_switch(req):
 #   global active_
  #  active_ = req.data
   # res = SetBoolResponse()
    #res.success = True
    #res.message = 'Done!!'
    #return res

def go_to_position_switch():
    global pub, active_, SetBool

    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
 
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
   # srv = rospy.Service('go_to_position_switch', SetBool, go_to_position_switch)
  
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/cylinder', rospy.Time(0))
            
            pos_x = trans[0]       
            pos_y = trans[1]
            pos_z = trans[2]
            print('pos_x = ', pos_x)
            print('pos_y = ', pos_y)
            print('pos_z = ', pos_z)
            break
            rospy.Rate(10)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

        

if __name__ == '__main__':
    try:
         go_to_position_switch()
    except rospy.ROSInterruptException:
         pass
