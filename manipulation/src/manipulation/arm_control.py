import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory, Grasp
import baxter_interface
from baxter_interface import CHECK_VERSION

class kmr19ArmCtrl:
  def __init__(self):
    self.robot = moveit_commander.RobotCommander()

    self.group = moveit_commander.MoveGroupCommander("both_arms")
    self.left_current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose
    self.right_current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose

    self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    self.left_gripper_close_pos = 0

    self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    self.right_gripper_close_pos = 0

  def moveToInitlPosition(self, arm='left'):
    if (arm == 'left'):
      #set target position
      target_pose = self.left_current_pose
      target_pose.position.x = 0.82
      target_pose.position.y = 0.3
      target_pose.position.z = 0.1
      target_pose.orientation.x = 0
      target_pose.orientation.y = 1
      target_pose.orientation.z = 0
      target_pose.orientation.w = 0

      #set target pose in MoveIt Commander
      self.group.set_pose_target(target_pose, end_effector_link='left_gripper')
    elif (arm == 'right'):
      #set target position
      target_pose = self.right_current_pose
      target_pose.position.x = 0.82
      target_pose.position.y = -0.3
      target_pose.position.z = 0.1
      target_pose.orientation.x = 0
      target_pose.orientation.y = 1
      target_pose.orientation.z = 0
      target_pose.orientation.w = 0
     
      #set target pose in MoveIt Commander
      self.group.set_pose_target(target_pose, end_effector_link='right_gripper')
    else:
      print("[kmr19ArmCtrl moveToInitlPosition]: specify which arm should be initialized")

    success = False
    for x in range(0, 4):
      # plan motion
      plan = self.group.plan()

      #check if plan was made
      if not plan.joint_trajectory.points:
        print("[kmr19ArmCtrl moveToInitlPosition] [ERROR] No trajectory found, retry....")
      else:
        #execute plan
        self.group.go(wait=True)
        self.group.stop()
        success = True
        break

    return success

  def planBlockPickup(self, block_pose, height_dif, arm='left'):
    #get current pose
    current_pose = self.left_current_pose
    link = 'left_gripper'
    if(arm == 'right'):
      current_pose = self.right_current_pose
      link = 'right_gripper'  

    #init target pose
    target_pose = current_pose

    # change orientation of target pose to bring gripper in right direction
    target_pose.orientation.x = 0
    target_pose.orientation.y = 1
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0

    # set x/y waypoint
    target_pose.position.x = block_pose.position.x
    target_pose.position.y = block_pose.position.y
    target_pose.position.z = block_pose.position.z + height_dif
    
    #set target pose in MoveIt Commander
    self.group.set_pose_target(target_pose, link)

    #compute path
    plan = self.group.plan()

    return plan

  def planBlockMovement(self, goal_pose, arm='left'):
    #get current pose
    current_pose = self.left_current_pose
    link = 'left_gripper'
    if(arm == 'right'):
      current_pose = self.right_current_pose
      link = 'right_gripper'

    #init target pose
    target_pose = current_pose

    # change orientation of target pose to bring gripper in right direction
    target_pose.orientation.x = 0
    target_pose.orientation.y = 1
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0

    # set x/y waypoint
    target_pose.position.x = goal_pose.position.x
    target_pose.position.y = goal_pose.position.y
    target_pose.position.z = goal_pose.position.z
    
    #set target pose in MoveIt Commander
    self.group.set_pose_target(target_pose, end_effector_link=link)

    #compute path
    plan = self.group.plan()

    return plan

  def executePlan(self, plan):
    success = False

    #check if plan is valid
    if not plan.joint_trajectory.points:
      print("[kmr19ArmCtrl executePlan]: given plan is not valid")
    else:
      #execute plan
      self.group.execute(plan, wait=True)
      self.group.stop()
      self.group.clear_pose_targets()
      success = True

    #set actual pose
    self.left_current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose
    self.right_current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose

    grasps = Grasp()
    grasps.grasp_pose.header.frame_id = "/base"
    grasps.grasp_pose.pose.position.x = 0.81
    grasps.grasp_pose.pose.position.y = 0.065
    grasps.grasp_pose.pose.position.z = -0.16
    grasps.grasp_pose.pose.orientation.x = 0
    grasps.grasp_pose.pose.orientation.y = 1
    grasps.grasp_pose.pose.orientation.z = 0
    grasps.grasp_pose.pose.orientation.w = 0
    grasps.allowed_touch_objects.append("table")

    grasps.pre_grasp_approach.direction.header.frame_id = "/base"
    grasps.pre_grasp_approach.direction.vector.z = 1.0
    grasps.pre_grasp_approach.min_distance = 0.01
    grasps.pre_grasp_approach.desired_distance = 0.02

    grasps.post_grasp_retreat.direction.header.frame_id = "/base"
    grasps.post_grasp_retreat.direction.vector.z = 1.0
    grasps.post_grasp_retreat.min_distance = 0.1
    grasps.post_grasp_retreat.desired_distance = 0.2

    self.group.pick("block_1", grasps)

    return success

  def initGripper(self, arm='left', gripper_open=True, block = True):
    #init left gripper
    if (arm == 'left'):
      self.left_gripper.reset(block=block)
      self.left_gripper.calibrate(block=block)
      self.left_gripper.set_holding_force(50)
      self.left_gripper.close(block=block)
      self.left_gripper_close_pos = self.left_gripper.position()

      if gripper_open:
        self.left_gripper.open(block=block)

      return True

    #init right gripper
    if (arm == 'right'):
      self.right_gripper.reset(block=block)
      self.right_gripper.calibrate(block=block)
      self.right_gripper.set_holding_force(50)
      self.right_gripper.close(block=block)
      self.right_gripper_close_pos = self.right_gripper.position()

      if gripper_open:
        self.right_gripper.open(block=block)

      return True

    #return False if parameter "arm" was wrong
    return False

  def pickupBlock(self, arm='left'):
    success = False    

    if(arm == 'left'):
      self.left_gripper.close(block=True)

      if(self.left_gripper.position() != self.left_gripper_close_pos):
        success = True

    elif(arm=='right'):
      self.right_gripper.close(block=True)

      if(self.right_gripper.position() != self.right_gripper_close_pos):
        success = True

    else:
      print("[kmr19ArmCtrl pickupBlock]: specify which arm should pickup block")

    return success

  def releaseBlock(self, arm='left'):
    if(arm == 'left'):
      self.left_gripper.open(block=True)
    if(arm == 'right'):
      self.right_gripper.open(block=True)

