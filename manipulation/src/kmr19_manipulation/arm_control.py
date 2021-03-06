
import rospy
import copy
import moveit_commander
import baxter_interface
from baxter_interface import CHECK_VERSION
from perception.msg import Block
import tf
from tf.transformations import *
from math import pi
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import OrientationConstraint
import numpy as np
from moveit_msgs.msg import RobotTrajectory

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

  def moveToInitPosition(self, arm='left'):
    if (arm == 'left'):
      self.left_gripper.open()
      self.l_holds_block = False
      group = self.group_l

      joint_goal = group.get_current_joint_values()
      joint_goal[0] = -pi/9
      joint_goal[1] = -pi/4
      joint_goal[2] = 0
      joint_goal[3] = pi/2
      joint_goal[4] = 0
      joint_goal[5] = pi/4.5
      joint_goal[6] = 0

      group.go(joint_goal, wait=True)
      group.stop()

      success = True
    elif (arm == 'right'):
      self.right_gripper.open()
      self.r_holds_block = False
      group = self.group_r

      joint_goal = group.get_current_joint_values()
      joint_goal[0] = pi/9
      joint_goal[1] = -pi/4
      joint_goal[2] = 0
      joint_goal[3] = pi/2
      joint_goal[4] = 0
      joint_goal[5] = pi/4.5
      joint_goal[6] = 0

      group.go(joint_goal, wait=True)
      group.stop()

      success = True
    else:
      print("[kmr19ArmCtrl moveToInitlPosition]: specify which arm should be initialized")
      success = False

    return success

  def planBlockPickup(self, block_pose, height_dif=0, arm='left', use_cartesian=False, use_constraint=False):
    '''
    :param block_pose: PoseStamped() of mid-point of block to pick up
    :param height_dif: height difference to block mid-point when closing gripper
    :param arm: defines which arm should be used
    :param use_cartesian: cartesian path is planned
    :param use_constraint: orientation constraint is set before planning starts
    :return: motion plan for pick up procedure, plan maybe not valid --> check somewhere else
    '''

    group = self.group_l
    link = 'left_gripper'
    frame = "left_gripper"
    if(arm == 'right'):
      group = self.group_r
      link = 'right_gripper'
      frame = "right_gripper"

    group.set_planning_time(15.0)

    #transform pose to right coordinate frame
    block_frame_id = block_pose.header.frame_id
    listener = tf.TransformListener()
    listener.waitForTransform("world", block_frame_id, rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform("world", block_frame_id, rospy.Time.now(), rospy.Duration(4.0))
    tmp_pose = listener.transformPose("world", block_pose)

    #init target pose
    target_pose = group.get_current_pose(end_effector_link=link)
    #save current pose
    current_pose = target_pose

    #change orientation of target pose to bring gripper in right direction
    q_orig = np.array([tmp_pose.pose.orientation.x, tmp_pose.pose.orientation.y, tmp_pose.pose.orientation.z, tmp_pose.pose.orientation.w])
    q_rot = quaternion_from_euler(-pi, 0, -pi)   # x = 0, y = 1, z = w = 0
    #apply rotation off gripper frame to block orientation
    q_new = quaternion_multiply(q_rot, q_orig)
    #set target orientation and position
    target_pose.pose.orientation.x = q_new[0]
    target_pose.pose.orientation.y = q_new[1]
    target_pose.pose.orientation.z = q_new[2]
    target_pose.pose.orientation.w = q_new[3]
    target_pose.header.stamp = rospy.Time(0)
    target_pose.pose.position.x = tmp_pose.pose.position.x
    target_pose.pose.position.y = tmp_pose.pose.position.y
    target_pose.pose.position.z = tmp_pose.pose.position.z + height_dif

    if use_constraint:
      #generate constraints and orientation constraint object
      constraints = Constraints()
      ocm = OrientationConstraint()

      #define orientation constraint
      ocm.header.frame_id = "/world"
      ocm.link_name = link
      ocm.orientation = target_pose.pose.orientation
      ocm.absolute_x_axis_tolerance = pi/8
      ocm.absolute_y_axis_tolerance = pi/8
      ocm.absolute_z_axis_tolerance = pi/8
      ocm.weight = 1

      #add constraint to planner
      constraints.orientation_constraints.append(ocm)
      group.set_path_constraints(constraints)

    if use_cartesian:
      #cartesian path waypoints
      waypoints = []
      #get detla z between current pose and target pose
      delta_z = current_pose.pose.position.z - target_pose.pose.position.z

      for i in range(5):
        #calc positions on path
        current_pose.pose.position.z -= delta_z/5
        #add target positions and generate second waypoint
        waypoints.append(copy.deepcopy(target_pose.pose))

      #compute paths until 100% of requirements are fullfilled
      for i in range(20):
        #compute path
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.001, 0.0)        
        if fraction == 1:
            break
      if fraction < 1:
        print("########################################")
        print("Cartesian Path failed")
        print("########################################")
        plan = RobotTrajectory()
    else:
      #set target pose in MoveIt Commander
      group.set_pose_target(target_pose.pose, link)

      plan = group.plan()

    return plan

  def executePlan(self, plan, arm='left', reduce_speed=False):
    success = False

    group = self.group_l
    link = 'left_gripper'
    if(arm == 'right'):
      group = self.group_r
      link = 'right_gripper'

    if reduce_speed:
      group.set_max_acceleration_scaling_factor(0.2)
      group.set_max_velocity_scaling_factor(0.2)

    #check if plan is valid
    if not plan.joint_trajectory.points:
      print("[kmr19ArmCtrl executePlan]: given plan is not valid")
    else:
      #execute plan
      group.execute(plan, wait=True)
      group.clear_pose_targets()
      group.clear_path_constraints()
      group.stop()
      success = True

    group.set_max_acceleration_scaling_factor(1)
    group.set_max_velocity_scaling_factor(1)

    return success

  def initGripper(self, arm='left', gripper_open=True, block=True):
    #init left gripper
    if (arm == 'left'):
      self.left_gripper.reboot()
      self.left_gripper.calibrate(block=block)
      self.left_gripper.close(block=block)
      
      rospy.sleep(1.0)
      for i in range(0, 10):
       self.left_gripper_close_pos += self.left_gripper.position()
       self.left_gripper_close_pos /= 10

      if gripper_open:
        self.left_gripper.open(block=block)

      return True

    #init right gripper
    if (arm == 'right'):
      self.right_gripper.reboot()
      self.right_gripper.calibrate(block=block)
      self.right_gripper.close(block=block)

      rospy.sleep(1.0)
      for i in range(0, 10):
       self.right_gripper_close_pos += self.right_gripper.position()
       self.right_gripper_close_pos /= 10

      if gripper_open:
        self.right_gripper.open(block=block)

      return True

    #return False if parameter "arm" was wrong
    return False

  def pickupBlock(self, arm='left'):
    success = False    

    if(arm == 'left'):
      self.left_gripper.set_holding_force(80.0)
      self.left_gripper.set_velocity(1.0)
      self.left_gripper.set_moving_force(80.0)

      for i in range(0, 80):
        self.left_gripper.command_position(position=float(100 - i), block=True)

      if(self.left_gripper.position() > self.left_gripper_close_pos*1.2):
        success = True

    elif(arm=='right'):
      self.right_gripper.set_holding_force(80.0)
      self.right_gripper.set_velocity(1.0)
      self.right_gripper.set_moving_force(80.0)

      for i in range(0, 101):
        self.right_gripper.command_position(position=float(100 - i), block=True)

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
      self.left_gripper.set_holding_force(80.0)
      self.left_gripper.set_velocity(1.0)
      self.left_gripper.set_moving_force(80.0)

      current_pos = self.left_gripper.position() + 1
      for i in range(int(current_pos), 101):
        self.left_gripper.command_position(position=float(i), block=True)

    if(arm == 'right'):
      self.right_gripper.set_holding_force(80.0)
      self.right_gripper.set_velocity(1.0)
      self.right_gripper.set_moving_force(80.0)
      
      current_pos = self.right_gripper.position() + 1
      for i in range(int(current_pos), 101):
        self.right_gripper.command_position(position=float(i), block=True)

    rospy.sleep(1)
