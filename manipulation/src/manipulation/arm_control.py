import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
import baxter_interface
from baxter_interface import CHECK_VERSION

class kmr19ArmCtrl:
  def __init__(self):
    self.group = moveit_commander.MoveGroupCommander("both_arms")
    self.left_current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose
    self.right_current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose
    self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)


  def moveToInitlPosition(self, arm='left'):
    if (arm == 'left'):
      #set target position
      target_pose = self.left_current_pose
      target_pose.position.x = 0.82
      target_pose.position.y = 0.065
      target_pose.position.z = 0.1
      target_pose.orientation.x = 0
      target_pose.orientation.y = 1
      target_pose.orientation.z = 0
      target_pose.orientation.w = 0

      #set target pose in MoveIt Commander
      self.group.set_pose_target(target_pose, end_effector_link='left_gripper')
    elif (arm == 'right'):
      #set target position
      '''
      target_pose = self.right_current_pose
      target_pose.position.x = 0.82
      target_pose.position.y = 0.065
      target_pose.position.z = 0.1
      target_pose.orientation.x = 0
      target_pose.orientation.y = 1
      target_pose.orientation.z = 0
      target_pose.orientation.w = 0
      '''
      target_pose = self.right_current_pose
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

  def planBlockPickup(self, block_pose, height_dif, eef_step, arm='left'):
    #cartesian path waypoints
    waypoints = []

    #execution plan
    plan = RobotTrajectory()
    fraction = 0

    #get current pose
    current_pose = self.left_current_pose
    if(arm == 'right'):
      current_pose = self.right_current_pose

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
    waypoints.append(copy.deepcopy(target_pose))

    # move down to block
    target_pose.position.z = block_pose.position.z + height_dif
    waypoints.append(copy.deepcopy(target_pose))

    #compute path
    (plan, fraction) = self.group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                         eef_step,  # eef_step
                                                         0.0)  # jump_threshold
    return plan, fraction

  def executePlan(self, plan):
    success = False

    #check if plan is valid
    if not plan.joint_trajectory.points:
      print("[kmr19ArmCtrl executePlan]: given plan is not valid")
    else:
      #execute plan
      self.group.go(wait=True)
      success = True

    #set actual pose
    self.left_current_pose = self.group.get_current_pose(end_effector_link='left_gripper').pose
    self.right_current_pose = self.group.get_current_pose(end_effector_link='right_gripper').pose

    return success

  def initGripper(self, arm='left', gripper_open=True):
    #init left gripper
    if (arm == 'left'):
      self.left_gripper.calibrate()
      if gripper_open:
        self.left_gripper.open()
      else:
        self.left_gripper.close()

      return True

    #init right gripper
    if (arm == 'right'):
      self.right_gripper.calibrate()
      if gripper_open:
        self.right_gripper.open()
      else:
        self.right_gripper.close()

      return True

    #return False if parameter "arm" was wrong
    return False

  def pickupBlock(self, arm='left'):
    success = False
    force = 0

    if(arm == 'left'):
      self.left_gripper.close()
      success = self.left_gripper.gripping()
      force = self.left_gripper.force()
    elif(arm=='right'):
      self.right_gripper.close()
      success = self.right_gripper.gripping()
      force = self.right_gripper.force()
    else:
      print("[kmr19ArmCtrl pickupBlock]: specify which arm should pickup block")

    print('Gripping Success and Force:', success, '  ', force)


