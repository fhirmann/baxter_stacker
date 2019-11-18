#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from manipulation.srv import kmr19_pick_up, kmr19_pick_upResponse, kmr19_put_down, kmr19_put_downResponse

import robot_control
import scene_control
import arm_control
from scene_control import scene_object

# predefine global control objects
robot_ctrl = 0
scene_ctrl = 0
arm_ctrl = 0

def handle_pick_up(req):
  """
  :param req: service request
    - Input:  uint64 block_id
    - Output: bool success
  :return: kmr19_pick_upResponse
  """
  global robot_ctrl, scene_ctrl, arm_ctrl

  print("[kmr19_manipulation_server]: Got request to pick up block with ID: ", req.block_id)

  #check if robot already holds block
  if robot_ctrl.hold_block:
    print("[kmr19_manipulation_server]: Robot already picked up a block")
    return kmr19_pick_upResponse(False)

  #check if block is in scene, return FAIL if not
  in_scene, block = scene_ctrl.isBlockInScene(req.block_id)
  if not in_scene:
    print("[kmr19_manipulation_server]: Block to pick up is not in scene")
    return kmr19_pick_upResponse(False)

  #plan and execute pre grasp procedure
  plan = arm_ctrl.planBlockPickup(block_pose=block.mid_pose, height_dif=0.05, arm='left')
  if not arm_ctrl.executePlan(plan):
    print("[kmr19_manipulation_server]: Robot failed during pre grasp pose")
    return kmr19_pick_upResponse(False)
  rospy.sleep(2)

  #remove block from planning scene
  removed = scene_ctrl.removeBlockFromPlanningScene(block_name=block.name)
  if not removed:
    print("[kmr19_manipulation_server]: Planning Scene error during pick up")
    return kmr19_pick_upResponse(False)

  #get motion plan for block pickup
  plan = arm_ctrl.planBlockPickup(block_pose=block.mid_pose, height_dif=0, arm='left')
  # execute plan
  if not arm_ctrl.executePlan(plan):
    print("[kmr19_manipulation_server]: Robot failed during grasp pose")
    return kmr19_pick_upResponse(False)
  rospy.sleep(1)

  # close gripper
  #gripped = arm_ctrl.pickupBlock(arm='left')
  arm_ctrl.pickupBlock(arm='left')
  gripped = True
  # add block to scene and attach to arm
  added = scene_ctrl.addBlockToPlanningScene(block)
  arm_ctrl.attachBlock(block=block, arm='left')

  if not added or not gripped:
    print("[kmr19_manipulation_server]: Planning Scene error during pick up")
    return kmr19_pick_upResponse(False)

  #plan and execute post grasp procedure
  plan = arm_ctrl.planBlockPickup(block_pose=block.mid_pose, height_dif=0.1, arm='left')
  if not arm_ctrl.executePlan(plan):
    print("[kmr19_manipulation_server]: Robot failed during post grasp pose")
    return kmr19_pick_upResponse(False)
  rospy.sleep(1)

  print("[kmr19_manipulation_server]: Block Pick-Up successful")
  return kmr19_pick_upResponse(True)

def handle_put_down(req):
  '''
  :param req: service request
    - Input:  geometry_msgs/PoseStamped end_position
    - Output: bool success
  :return:
  '''
  global robot_ctrl, scene_ctrl, arm_ctrl

  print("[kmr19_manipulation_server]: Got request to put down block at", req.end_position.pose.position.x, req.end_position.pose.position.y, req.end_position.pose.position.z)

  #check if robot already holds block
  if not robot_ctrl.hold_block:
    print("[kmr19_manipulation_server]: Robot has to pick up a block first")
    return kmr19_put_downResponse(False)

def kmr19_manipulation_server():
  global robot_ctrl, scene_ctrl, arm_ctrl

  # initialize moveit_commander
  joint_state_topic = ['joint_states:=/robot/joint_states']
  moveit_commander.roscpp_initialize(joint_state_topic)

  #init manipulation server node
  rospy.init_node('kmr19_manipulation_server')

  # set global control objects
  robot_ctrl = robot_control.kmr19RobotCtrl()
  scene_ctrl = scene_control.kmr19SceneControl()
  arm_ctrl = arm_control.kmr19ArmCtrl()

  #enable robot
  robot_ctrl.enableRobot()

  #add fixed scene object
  scene_ctrl.addFixedSceneObjects()

  #init position and gripper init
  left_arm_init = arm_ctrl.moveToInitlPosition(arm='left')
  left_gripper_init = arm_ctrl.initGripper(arm='left', gripper_open=True, block=True)
  right_arm_init = arm_ctrl.moveToInitlPosition(arm='right')
  right_gripper_init = arm_ctrl.initGripper(arm='right', gripper_open=True, block=True)

  print("[kmr19_manipulation_server]: Left arm init successful? ", left_arm_init)
  print("[kmr19_manipulation_server]: Left gripper init successful? ", left_gripper_init)
  print("[kmr19_manipulation_server]: Right arm init successful? ", right_arm_init)
  print("[kmr19_manipulation_server]: Right gripper init successful? ", right_gripper_init)

  #define services the server runs
  rospy.Service('kmr19_manipulation_pick_up', kmr19_pick_up, handle_pick_up)
  rospy.Service('kmr19_manipulation_put_down', kmr19_put_down, handle_put_down)

  print("kmr19_manipulation_server ready")

  rospy.spin()
