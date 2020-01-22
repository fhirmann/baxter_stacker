#!/usr/bin/env python

#TODO: Code cleanup (arm_control)
#TODO: Test Fabian quarternion stuff
#TODO: When cartesian path not possible --> endless loop

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from manipulation.srv import kmr19_pick_up, kmr19_pick_upResponse, kmr19_put_down, kmr19_put_downResponse

import robot_control
import scene_control
import arm_control

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

  error_code = kmr19_pick_upResponse.NO_ERROR

  print("[kmr19_manipulation_server]: Got request to pick up block with ID: ", req.block_id)

  #load scene from data base
  scene_ctrl.loadSceneDatabase()

  #check if robot already holds block
  if arm_ctrl.l_holds_block:
    print("[kmr19_manipulation_server]: Robot already picked up a block")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.HOLD_BLOCK
    return kmr19_pick_upResponse(success=False, error_code=error_code)

  #check if block is in scene, return FAIL if not
  in_scene, block = scene_ctrl.isBlockInScene(req.block_id)
  if not in_scene:
    print("[kmr19_manipulation_server]: Block to pick up is not in scene")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.BLOCK_NOT_FOUND
    return kmr19_pick_upResponse(success=False, error_code=error_code)

  h_dif = block.height*1.2
  
  #plan and execute pre grasp procedure
  plan = arm_ctrl.planBlockPickup(block_pose=block.pose, height_dif=h_dif, arm='left')
  if not arm_ctrl.executePlan(plan):
    print("[kmr19_manipulation_server]: Robot failed during pre grasp pose")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.PLAN_PRE_GRASP
    return kmr19_pick_upResponse(success=False, error_code=error_code)
  rospy.sleep(1.5)

  #remove block from planning scene
  removed = scene_ctrl.removeBlockFromPlanningScene(block_name=str(block.id))
  if not removed:
    print("[kmr19_manipulation_server]: Planning Scene error during pick up")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.MOVEIT_SCENE_ERROR
    return kmr19_pick_upResponse(success=False, error_code=error_code)

  #get motion plan for block pickup
  print("DO GRASP")
  plan = arm_ctrl.planBlockPickup(block_pose=block.pose, height_dif=0, arm='left', use_constraint=False, use_cartesian=True)
  # execute plan
  if not arm_ctrl.executePlan(plan, reduce_speed=True):
    print("[kmr19_manipulation_server]: Robot failed during grasp pose")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.PLAN_GRASP
    return kmr19_pick_upResponse(success=False, error_code=error_code)
  rospy.sleep(1)
  print("GRASP DONE")
  # close gripper
  gripped = arm_ctrl.pickupBlock(arm='left')
  # add block to scene and attach to arm
  added = scene_ctrl.addBlockToPlanningScene(block)
  arm_ctrl.attachBlock(block=block, arm='left')

  if not added or not gripped:
    print("[kmr19_manipulation_server]: Planning Scene error during pick up")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.MOVEIT_SCENE_ERROR
    return kmr19_pick_upResponse(success=False, error_code=error_code)

  #plan and execute post grasp procedure
  plan = arm_ctrl.planBlockPickup(block_pose=block.pose, height_dif=h_dif, arm='left', use_constraint=False, use_cartesian=True)
  if not arm_ctrl.executePlan(plan, reduce_speed=True):
    print("[kmr19_manipulation_server]: Robot failed during post grasp pose")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_pick_upResponse.PLAN_POST_GRASP
    return kmr19_pick_upResponse(success=False, error_code=error_code)
  rospy.sleep(1)

  #set arm block status
  arm_ctrl.l_holds_block = True
  arm_ctrl.l_block = block

  print("[kmr19_manipulation_server]: Block Pick-Up successful")
  return kmr19_pick_upResponse(success=True, error_code=error_code)

def handle_put_down(req):
  '''
  :param req: service request
    - Input:  geometry_msgs/PoseStamped end_position
    - Output: bool success
  :return:
  '''
  global robot_ctrl, scene_ctrl, arm_ctrl

  error_code = kmr19_put_downResponse.NO_ERROR

  print("[kmr19_manipulation_server]: Got request to put down block at", req.end_position.pose.position.x, req.end_position.pose.position.y, req.end_position.pose.position.z)
  print("in frame ", req.end_position.header.frame_id)

  #workaround for quarternion
  #req.end_position.pose.orientation.w = 1.0

  #check if robot already holds block
  if not arm_ctrl.l_holds_block:
    print("[kmr19_manipulation_server]: Robot has to pick up a block first")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_put_downResponse.HOLD_NO_BLOCK
    return kmr19_put_downResponse(success=False, error_code=error_code)

  h_dif = arm_ctrl.l_block.height*1.2

  #plan and execute pre release procedure
  #plan = arm_ctrl.planBlockPutdown(goal_pose=req.end_position, height_dif=h_dif, arm='left')
  plan = arm_ctrl.planBlockPickup(block_pose=req.end_position, height_dif=h_dif, arm='left')
  if not arm_ctrl.executePlan(plan):
    print("[kmr19_manipulation_server]: Robot failed during pre release pose")
    arm_ctrl.moveToInitPosition(arm='left')
    arm_ctrl.detachBlock(block_name=str(arm_ctrl.l_block.id))
    error_code = kmr19_put_downResponse.PLAN_PRE_RELEASE
    return kmr19_put_downResponse(success=False, error_code=error_code)
  rospy.sleep(1.5)

  #detach block
  arm_ctrl.detachBlock(block_name=str(arm_ctrl.l_block.id))

  # remove block from planning scene
  removed = scene_ctrl.removeBlockFromPlanningScene(block_name=str(arm_ctrl.l_block.id))
  if not removed:
    print("[kmr19_manipulation_server]: Planning Scene error during put down")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_put_downResponse.MOVEIT_SCENE_ERROR
    return kmr19_put_downResponse(success=False, error_code=error_code)

  # plan and execute release procedure
  #plan = arm_ctrl.planBlockPutdown(goal_pose=req.end_position, height_dif=0.0, arm='left', use_constraint=False, use_cartesian=True)
  plan = arm_ctrl.planBlockPickup(block_pose=req.end_position, height_dif=0.005, arm='left', use_cartesian=True)
  if not arm_ctrl.executePlan(plan, reduce_speed=True):
    print("[kmr19_manipulation_server]: Robot failed during release pose")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_put_downResponse.PLAN_RELEASE
    return kmr19_put_downResponse(success=False, error_code=error_code)
  rospy.sleep(1)

  #release block
  arm_ctrl.releaseBlock()

  # plan and execute post release procedure
  #plan = arm_ctrl.planBlockPutdown(goal_pose=req.end_position, height_dif=h_dif, arm='left', use_constraint=False, use_cartesian=True)
  plan = arm_ctrl.planBlockPickup(block_pose=req.end_position, height_dif=h_dif, arm='left', use_cartesian=True)
  if not arm_ctrl.executePlan(plan, reduce_speed=True):
    print("[kmr19_manipulation_server]: Robot failed during post release pose")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_put_downResponse.PLAN_POST_RELEASE
    return kmr19_put_downResponse(success=False, error_code=error_code)
  rospy.sleep(0.5)

  #change position of block
  arm_ctrl.l_block.pose = req.end_position

  # add block to scene
  if not scene_ctrl.addBlockToPlanningScene(arm_ctrl.l_block):
    print("[kmr19_manipulation_server]: Planning Scene error during put down")
    arm_ctrl.moveToInitPosition(arm='left')
    error_code = kmr19_put_downResponse.MOVEIT_SCENE_ERROR
    return kmr19_put_downResponse(success=False, error_code=error_code)

  #update block position in scene_control
  scene_ctrl.updateBlockInScene(arm_ctrl.l_block)

  #move back to init position
  arm_ctrl.moveToInitPosition(arm='left')

  arm_ctrl.l_holds_block = False
  print("[kmr19_manipulation_server]: Block Put-Down successful")
  return kmr19_put_downResponse(success=True, error_code=error_code)

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

  #set head position
  robot_ctrl.initHead()

  #add fixed scene object
  scene_ctrl.addFixedSceneObjects()

  #init position and gripper init
  left_arm_init = arm_ctrl.moveToInitPosition(arm='left')
  left_gripper_init = arm_ctrl.initGripper(arm='left', gripper_open=True, block=True)
  right_arm_init = arm_ctrl.moveToInitPosition(arm='right')
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
