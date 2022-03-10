#!/usr/bin/env python
   
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Twist
from math import atan2
from tf import TransformListener
import tf
import math
#mport cv2 

import numpy as np 

x = 0.0
y = 0.0 
theta = 0.0
pos_x1 = 0.0
pos_y1 = 0.0
pos_w1 = 0.0
pos_x2 = 0.0
pos_y2 = 0.0
pos_w2 = 0.0

def newOdom(msg):
    
    global x
    global y
    global w
    global theta
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    #print("current x position", x)
    #print("current y position", y)

    rot_q = msg.pose.pose.orientation
    #print(rot_q)
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #print"convert rad to degree"
    #print(np.degrees(theta))
    
def mobile_Crusher(callback):
 
    print "!!!!! ======== goal positions for mobile crusher ======== !!!!!"    
    
    global pos_x2, pos_y2, pos_w2
 
    pub = rospy.Publisher("/mc_robo/cmd_vel", Twist, queue_size = 1)
    listener = tf.TransformListener()    
    r = rospy.Rate(4)

    while not rospy.is_shutdown():
        try:

            (trans,rot) = listener.lookupTransform('/base_link', '/mobile_crusher', rospy.Time(0))
            pos_x2 = trans[0]
            pos_y2 = trans[1]
            pos_w2 = rot[3]
            print("goal position_x for mc", pos_x2)
            print("goal position_y for mc", pos_y2)
            print("quaternion pos_w for mc", pos_w2)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

def main():
    
    global pos_x1, pos_y1, pos_w1

    print "!!!!! ======== Start program ======== !!!!!"
    rospy.init_node("speed_controller")
   
    r = rospy.Rate(4)
    sub = rospy.Subscriber("/bw_robo/odom", Odometry, newOdom)
    print"finding location in Gazebo-------"
    pub = rospy.Publisher("/bw_robo/cmd_vel", Twist, queue_size = 1)

    print"listening postion from tf_listener"
    print"getting kinematic chain position and pass it to Gazebo"
    listener = tf.TransformListener()    
    r = rospy.Rate(4)

    turn_left = 0
    turn_right = 0
    while not rospy.is_shutdown():
        try:
            
            (trans,rot) = listener.lookupTransform('/base_link', '/cylinder', rospy.Time(0))
            pos_x1 = trans[0]       
            pos_y1 = trans[1]
            pos_w1 = rot[3]
            
            speed = Twist()

            goal = Point()
            goal.x = pos_x1
            goal.y = pos_y1
	    goal.z = pos_w1
            print("goal position_x for bw", goal.x)
            print("goal position_y for bw", goal.y)
	    #print("quaternion pos_w for bw", goal.z)


            inc_x = goal.x -x
            inc_y = goal.y -y
  
            angle_to_goal = atan2(inc_y, inc_x)
            angle_diff = np.degrees(angle_to_goal - theta)
            print(angle_diff)
            print("current x position", x)
            print("current y position", y)

            
            if angle_diff < -10:
               print"---- turn left ----"
               turn_left = 1

	    elif angle_diff > -5:
               turn_left = 0

            if angle_diff > 10:
               print"---- turn right ----"
               turn_right = 1

            elif angle_diff < 5:
               turn_right = 0
       
            if (goal.x - 0.2 < x < goal.x + 0.2) and (goal.y - 0.2 < y < goal.y + 0.2):
 	       print"---- Goal Reached ----"
               speed.linear.x = 0.0
               speed.angular.z = 0.0

            elif turn_left == 1:
               speed.linear.x = 0.0
               speed.angular.z = 1.0 # clockwise

            elif turn_right == 1:
               speed.linear.x = 0.0
               speed.angular.z = -1.0 # counter clockwise
            
            else:
               print"go forward"
               speed.linear.x = 0.5
               speed.angular.z = 0.0



            pub.publish(speed)
            r.sleep() 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
           

if __name__ == '__main__':
    try:
         main()
    except rospy.ROSInterruptException:
         pass
