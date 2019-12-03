#!/usr/bin/env python

#TODO: Block Message depth --> x coordinate, width --> y coordinate

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

from mongodb_store.message_store import MessageStoreProxy
from perception.msg import Block

class kmr19SceneControl:
  def __init__(self):
    self.scene = moveit_commander.PlanningSceneInterface()
    self.scene_blocks = []
    self.robot = moveit_commander.RobotCommander()

  def addFixedSceneObjects(self):
    rospy.sleep(0.1)  #sleep needed for scene manipulation
    #clear scene
    self.scene.remove_world_object()
    self.scene.remove_attached_object('left_gripper')
    self.scene.remove_attached_object('right_gripper')
    if not self.checkEmptyScene(10):
      print("[kmr19SceneCtrl addFixedSceneObjects]: Not able to clear scene!")
    rospy.sleep(0.1)

    #add table to MoveIt scene only
    table_pose = PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.position.x = 0.93
    table_pose.pose.position.y = 0.3
    table_pose.pose.position.z = -0.55
    table_size = (0.9, 1.6, 0.7)
    # add to MoveIt scene
    self.scene.add_box("table", table_pose, table_size)
    # wait until scene is changed
    self.checkObject(3, "table", obj_is_attached=False, obj_is_known=True)

    print("[kmr19SceneCtrl addFixedSceneObjects]: added fixed scene objects!")

  def isBlockInScene(self, id):
    for index, item in enumerate(self.scene_blocks):
      if item.id == id:
        return True, item

    return False, Block()

  def updateBlockInScene(self, block):
    for index, item in enumerate(self.scene_blocks):
      if item.id == block.id:
        item = block

  def removeBlockFromPlanningScene(self, block_name=""):
    self.scene.remove_world_object(block_name)
    return self.checkObject(3, block_name, obj_is_attached=False, obj_is_known=False)

  def addBlockToPlanningScene(self, block):
    name = str(block.id)
    # add to MoveIt scene
    self.scene.add_box(name, block.pose, size=(block.depth, block.width, block.height))
    # wait until scene is changed
    return self.checkObject(3, name, obj_is_attached=False, obj_is_known=True)

  def checkObject(self, timeout, name, obj_is_attached=False, obj_is_known=False):
    #copied from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (obj_is_attached == is_attached) and (obj_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def checkEmptyScene(self, timeout):
    #copied from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if we are in the expected state
      if ( len( self.scene.get_objects() ) == 0 ):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def loadSceneDatabase(self):
    msg_store = MessageStoreProxy()

    #empty scene objects
    del self.scene_blocks[:]
    # get all blocks from
    data = msg_store.query(Block._type)

    #add blocks from database to MoveIt scene
    for (block, meta) in data:
      self.scene_blocks.append(block)
      self.addBlockToPlanningScene(block=block)




