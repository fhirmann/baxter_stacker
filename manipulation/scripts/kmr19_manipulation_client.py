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
  pickup(1)

  block_pose = PoseStamped()
  block_pose.header.frame_id = "/table"

  #### STACK
  block_pose.pose.position.x = 0.24
  block_pose.pose.position.y = 0.7
  block_pose.pose.position.z = 0.04
  block_pose.pose.orientation.w = 1.0

  result = putdown(block_pose)
  print "kmr pit down successful? ", result



