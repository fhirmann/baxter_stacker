#!/usr/bin/env python

import sys
import rospy
from manipulation.srv import *
from geometry_msgs.msg import PoseStamped

def pickup(id):
  rospy.wait_for_service('kmr19_manipulation_pick_up')
  try:
    pick_up = rospy.ServiceProxy('kmr19_manipulation_pick_up', kmr19_pick_up)
    resp1 = pick_up(id)
    return resp1.success
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def putdown(pose):
  rospy.wait_for_service('kmr19_manipulation_put_down')
  try:
    put_down = rospy.ServiceProxy('kmr19_manipulation_put_down', kmr19_put_down)
    resp1 = put_down(pose)
    return resp1.success
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

if __name__ == "__main__":
  success = pickup(2)
  print "kmr pick up successful? ", success

  block_pose = PoseStamped()
  block_pose.header.frame_id = "/world"

  #### STACK
  block_pose.pose.position.x = 0.785
  block_pose.pose.position.y = 0.075
  block_pose.pose.position.z = -0.08

  #### UNSTACK
  block_pose.pose.position.x = 0.785
  block_pose.pose.position.y = 0.4 #= 0.08
  block_pose.pose.position.z = -0.16

  success = putdown(block_pose)
  print "kmr pit down successful? ", success


