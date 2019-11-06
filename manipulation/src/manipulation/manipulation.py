#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped

import robot_control
import scene_control
import arm_control

def main():
  # initialize moveit_commander and rospy.
  joint_state_topic = ['joint_states:=/robot/joint_states']
  moveit_commander.roscpp_initialize(joint_state_topic)
  rospy.init_node('baxter_manipulation_node', anonymous=False)

  #generate control objects
  robot_ctrl = robot_control.kmr19RobotCtrl()
  scene_ctrl = scene_control.kmr19SceneControl()
  arm_ctrl = arm_control.kmr19ArmCtrl()

  #enable robot
  robot_ctrl.enableRobot()

  #add fixed scene objects
  rospy.sleep(2)  #sleep is necessary to add scene objects
  scene_ctrl.addFixedSceneObjects()

  #init position and gripper init
  arm_ctrl.moveToInitlPosition(arm='left')
  arm_ctrl.initGripper(arm='left', gripper_open=True)

  #block pose
  block_pose = Pose()
  block_pose.position.x = 0.82
  block_pose.position.y = 0.065
  block_pose.position.z = -0.16

  #plan block pickup
  (plan, fraction) = arm_ctrl.planBlockPickup(block_pose, 0, 0.01, arm='left')

  #execute plan
  arm_ctrl.executePlan(plan)
  arm_ctrl.pickupBlock()

  # When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  moveit_commander.os._exit(0)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
