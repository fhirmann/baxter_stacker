#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped

import robot_control
import scene_control
import arm_control

from scene_control import scene_object

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

  #add fixed scene object
  scene_ctrl.addFixedSceneObjects()

  #init position and gripper init
  arm_ctrl.moveToInitlPosition(arm='left')
  arm_ctrl.initGripper(arm='left', gripper_open=True, block=False)
  arm_ctrl.moveToInitlPosition(arm='right')
  arm_ctrl.initGripper(arm='right', gripper_open=True, block=False)

  #block pose
  block_pose = Pose()
  block_pose.position.x = 0.81
  block_pose.position.y = 0.065
  block_pose.position.z = -0.11
  
  #plan block pickup
  plan = arm_ctrl.planBlockPickup(block_pose, 0, arm='left')
  #execute plan
  arm_ctrl.executePlan(plan)

  rospy.sleep(2)
  arm_ctrl.pickupBlock(arm='left')

  '''
  scene_block = scene_object()
  scene_block.name = "block_1"
  scene_block.color = "yellow"
  scene_block.mid_pose.header.frame_id = "/base"
  scene_block.mid_pose.pose.position.x = 0.82
  scene_block.mid_pose.pose.position.y = 0.065
  scene_block.mid_pose.pose.position.z = -0.16 
  scene_block.size = (0.04, 0.04, 0.08)
  scene_ctrl.addBlockToScene(scene_block)
  scene_ctrl.attachBlockToArm(arm='left')

  #new block pose
  block_pose = Pose()
  block_pose.position.x = 0.82
  block_pose.position.y = 0.3
  block_pose.position.z = 0.1

  #plan block movement
  plan = arm_ctrl.planBlockMovement(block_pose, arm='left')

  #execute plan
  arm_ctrl.executePlan(plan)
  rospy.sleep(2)
  arm_ctrl.releaseBlock(arm='left')
  '''
  # When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  moveit_commander.os._exit(0)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
