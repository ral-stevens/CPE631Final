#!/usr/bin/env python

import rospy 
import rospkg 
import argparse
import math

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_robot_state')

    parser = argparse.ArgumentParser(description="Set robot model state.")
    parser.add_argument("x", help="Specify the x coordinate.")
    parser.add_argument("y", help="Specify the y coordinate.")
    parser.add_argument("th", help="Specify the orientation in radians.")

    args = parser.parse_args()
    x = float(args.x)
    y = float(args.y)
    th = float(args.th)

    state_msg = ModelState()
    state_msg.model_name = 'my_robot'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = math.sin(th/2)
    state_msg.pose.orientation.w = math.cos(th/2)

    rospy.loginfo('waiting for service /gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        rospy.loginfo('%s' % resp)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass