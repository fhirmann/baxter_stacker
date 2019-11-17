#!/usr/bin/env python

from kmr19_manipulation import kmr19_manipulation_server

if __name__== '__main__':
     kmr19_manipulation_server()

'''
from manipulation.srv import kmr19_pick_up, kmr19_pick_upResponse, kmr19_put_down, kmr19_put_downResponse
import rospy

def handle_pick_up():
  print("Pick up")
  script_imprt()
  pass

def handle_put_down():
  print("Put down")
  pass

def kmr19_manipulation_server():
  # init manipulation server node
  rospy.init_node('kmr19_manipulation_server')

  # define services the server runs
  rospy.Service('kmr19_manipulation_pick_up', kmr19_pick_up, handle_pick_up)
  rospy.Service('kmr19_manipulation_put_down', kmr19_put_down, handle_put_down)

  print("kmr19_manipulation_server ready")
  rospy.spin()

if __name__== '__main__':
     kmr19_manipulation_server()
'''
