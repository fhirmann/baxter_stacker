
import rospy
import copy
import moveit_commander
import baxter_interface
from baxter_interface import CHECK_VERSION
from perception.msg import Block

class kmr19ArmCtrl:
  def __init__(self):
    self.group = moveit_commander.MoveGroupCommander("both_arms")
    self.group_l = moveit_commander.MoveGroupCommander("left_arm")
    self.group_r = moveit_commander.MoveGroupCommander("right_arm")

    self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    self.left_gripper_close_pos = 0

    self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    self.right_gripper_close_pos = 0

    self.robot = moveit_commander.RobotCommander()

    self.l_holds_block = False
    self.r_holds_block = False
    self.l_block = Block()
    self.r_block = Block()

  def moveToInitlPosition(self, arm='left'):
    if (arm == 'left'):
      group = self.group_l
      link = 'left_gripper'

      #set target position
      target_pose = group.get_current_pose(end_effector_link=link).pose

      target_pose.position.x = 0.82
      target_pose.position.y = 0.3
      target_pose.position.z = 0.1
      target_pose.orientation.x = 0
      target_pose.orientation.y = 1
      target_pose.orientation.z = 0
      target_pose.orientation.w = 0

    elif (arm == 'right'):
      group = self.group_r
      link = 'right_gripper'

      #set target position
      target_pose = group.get_current_pose(end_effector_link=link).pose
      target_pose.position.x = 0.82
      target_pose.position.y = -0.3
      target_pose.position.z = 0.1
      target_pose.orientation.x = 0
      target_pose.orientation.y = 1
      target_pose.orientation.z = 0
      target_pose.orientation.w = 0
    else:
      print("[kmr19ArmCtrl moveToInitlPosition]: specify which arm should be initialized")

    # set target pose in MoveIt Commander
    group.set_pose_target(target_pose, end_effector_link=link)

    success = False
    for x in range(0, 4):
      # plan motion
      plan = self.group.plan()

      #check if plan was made
      if not plan.joint_trajectory.points:
        print("[kmr19ArmCtrl moveToInitlPosition] [ERROR] No trajectory found, retry....")
      else:
        #execute plan
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        success = True
        break

    return success

  def planBlockPickup(self, block_pose, height_dif=0, arm='left'):
    '''
    :param block_pose: PoseStamped() of mid-point of block to pick up
    :param height_dif: height difference to block mid-point when closing gripper
    :param arm: defines which arm should be used
    :return: motion plan for pick up procedure, plan maybe not valid --> check somewhere else
    '''

    group = self.group_l
    link = 'left_gripper'
    frame = "left_gripper"
    if(arm == 'right'):
      group = self.group_r
      link = 'right_gripper'
      frame = "right_gripper"

    #cartesian path waypoints
    waypoints = []

    #init target pose
    target_pose = group.get_current_pose(end_effector_link=link).pose

    #change orientation of target pose to bring gripper in right direction
    target_pose.orientation.x = 0
    target_pose.orientation.y = 1
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0

    #generate first waypoint
    target_pose.position.x = block_pose.pose.position.x
    target_pose.position.y = block_pose.pose.position.y
    target_pose.position.z = block_pose.pose.position.z + height_dif/2

    #add target positions and generate second waypoint
    waypoints.append(copy.deepcopy(target_pose))
    target_pose.position.z = block_pose.pose.position.z + height_dif/2
    waypoints.append(copy.deepcopy(target_pose))


    #set target pose in MoveIt Commander
    #group.set_pose_target(target_pose, link)

    #compute path
    (plan, fraction) = group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                    0.01,       # eef_step
                                                    0.0)        # jump_threshold

    #plan = group.plan()

    return plan

  def planBlockPutdown(self, goal_pose, height_dif=0, arm='left'):
    '''
       :param block_pose: PoseStamped() of mid-point of block to pick up
       :param height_dif: height difference to block mid-point when closing gripper
       :param arm: defines which arm should be used
       :return: motion plan for pick up procedure, plan maybe not valid --> check somewhere else
       '''

    group = self.group_l
    link = 'left_gripper'
    if (arm == 'right'):
      group = self.group_r
      link = 'right_gripper'

    # cartesian path waypoints
    waypoints = []

    # init target pose
    target_pose = group.get_current_pose(end_effector_link=link).pose

    # change orientation of target pose to bring gripper in right direction
    target_pose.orientation.x = 0
    target_pose.orientation.y = 1
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0

    target_pose.position.x = goal_pose.position.x
    target_pose.position.y = goal_pose.position.y
    target_pose.position.z = goal_pose.position.z + height_dif/2

    # add target position
    waypoints.append(copy.deepcopy(target_pose))
    target_pose.position.z = goal_pose.position.z + height_dif/2
    waypoints.append(copy.deepcopy(target_pose))

    # set target pose in MoveIt Commander
    # group.set_pose_target(target_pose, link)

    # compute path
    (plan, fraction) = group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                    0.001,  # eef_step
                                                    0.0)  # jump_threshold

    # plan = group.plan()

    return plan

  def executePlan(self, plan, arm='left'):
    success = False

    group = self.group_l
    link = 'left_gripper'
    if(arm == 'right'):
      group = self.group_r
      link = 'right_gripper'

    #check if plan is valid
    if not plan.joint_trajectory.points:
      print("[kmr19ArmCtrl executePlan]: given plan is not valid")
    else:
      #execute plan
      group.execute(plan, wait=True)
      group.stop()
      group.clear_pose_targets()
      success = True

    return success

  def initGripper(self, arm='left', gripper_open=True, block=True):
    #init left gripper
    if (arm == 'left'):
      self.left_gripper.reboot()
      self.left_gripper.calibrate(block=block)
      self.left_gripper.close(block=block)
      self.left_gripper_close_pos = self.left_gripper.position()

      if gripper_open:
        self.left_gripper.open(block=block)

      return True

    #init right gripper
    if (arm == 'right'):
      self.right_gripper.reboot()
      self.right_gripper.calibrate(block=block)
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
      #for i in range(0, 100):
      #  self.left_gripper.command_position(position=float(100-i), block=True)
      self.left_gripper.set_holding_force(50.0)
      self.left_gripper.set_velocity(1.0)
      self.left_gripper.set_moving_force(1.0)
      self.left_gripper.close(block=True)
      #print("Close Position: ", self.left_gripper_close_pos)
      if(self.left_gripper.position() > self.left_gripper_close_pos*1.2):
        success = True

    elif(arm=='right'):
      #for i in range(0, 50):
      #  self.left_gripper.command_position(position=float(100-i), block=True)
      self.right_gripper.set_holding_force(50.0)
      self.right_gripper.set_velocity(1.0)
      self.right_gripper.set_moving_force(1.0)
      self.right_gripper.close(block=True)
      if(self.right_gripper.position() > self.right_gripper_close_pos*1.2):
        success = True
    else:
      print("[kmr19ArmCtrl pickupBlock]: specify which arm should pickup block")

    rospy.sleep(1)
    return success

  def attachBlock(self, block, arm='left'):
    group = self.group_l
    eef_link = 'left_gripper'

    touch_links = self.robot.get_link_names(group='left_arm')
    touch_links.append('l_gripper_l_finger')
    touch_links.append('l_gripper_r_finger')
    touch_links.append('l_gripper_l_finger_tip')
    touch_links.append('l_gripper_r_finger_tip')

    if(arm == 'right'):
      group = self.group_r
      eef_link = 'right_gripper'

      touch_links = self.robot.get_link_names(group='right_arm')
      touch_links.append('r_gripper_l_finger')
      touch_links.append('r_gripper_r_finger')
      touch_links.append('r_gripper_l_finger_tip')
      touch_links.append('r_gripper_r_finger_tip')

    group.attach_object(link_name=eef_link, object_name=str(block.id), touch_links=touch_links)

  def detachBlock(self, block_name="", arm='left'):
    group = self.group_l
    if (arm == 'right'):
      group = self.group_r

    group.detach_object(block_name)

  def releaseBlock(self, arm='left'):
    if(arm == 'left'):
      #current_pos = self.left_gripper.position() + 1
      #for i in range(int(current_pos), 101):
      #  self.left_gripper.command_position(position=float(i), block=True)
      self.left_gripper.set_holding_force(50.0)
      self.left_gripper.set_velocity(1.0)
      self.left_gripper.set_moving_force(1.0)
      self.left_gripper.open(block=True)

    if(arm == 'right'):
      #current_pos = self.right_gripper.position() + 1
      #for i in range(int(current_pos), 101):
      #  self.right_gripper.command_position(position=float(i), block=True)
      self.right_gripper.set_holding_force(50.0)
      self.right_gripper.set_velocity(1.0)
      self.right_gripper.set_moving_force(1.0)
      self.right_gripper.open(block=True)

    rospy.sleep(1)
