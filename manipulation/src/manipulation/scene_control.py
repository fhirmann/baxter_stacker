#!/usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

class kmr19SceneControl:
  def __init__(self):
    self.scene = moveit_commander.PlanningSceneInterface()
    self.scene_blocks = []

  def addFixedSceneObjects(self):
    rospy.sleep(0.1)  #sleep needed for scene manipulation
    #clear scene
    self.scene.remove_world_object()
    self.scene.remove_attached_object('left_gripper')
    self.scene.remove_attached_object('right_gripper')
    if not self.checkEmptyScene(10):
      print("[kmr19SceneCtrl addFixedSceneObjects]: Not able to clear scene!")
    rospy.sleep(0.1)

    new_obj = scene_object()    
    new_obj.name = "table"
    new_obj.color = "white"
    new_obj.mid_pose.header.frame_id = "/base"
    new_obj.mid_pose.pose.position.x = 0.93
    new_obj.mid_pose.pose.position.y = 0.3
    new_obj.mid_pose.pose.position.z = -0.55 
    new_obj.size = (0.8, 1.6, 0.7)
    self.addBlockToScene(new_obj)
    #self.scene.attach_box('base', "table", touch_links=['pedestal'])
    #self.checkObject(10, "table", obj_is_attached=True, obj_is_known=False)

    new_obj = scene_object()    
    new_obj.name = "block_1"
    new_obj.color = "yellow"
    new_obj.mid_pose.header.frame_id = "/base"
    new_obj.mid_pose.pose.position.x = 0.82
    new_obj.mid_pose.pose.position.y = 0.065
    new_obj.mid_pose.pose.position.z = -0.16 
    new_obj.size = (0.04, 0.04, 0.08)
    self.addBlockToScene(new_obj)

    print("[kmr19SceneCtrl addFixedSceneObjects]: added fixed scene objects!")

  def addBlockToScene(self, block):
    #add block to SceneControl
    self.scene_blocks.append(block)
    #add to MoveIt scene
    self.scene.add_box( block.name, block.mid_pose, block.size )
    #wait until scene is changed
    self.checkObject(10, block.name, obj_is_attached=False, obj_is_known=True)

  def attachBlockToArm(self, arm='left'):
    if(arm=='left'):
      eef_link = 'left_gripper'
    if(arm=='right'):
      eef_link = 'right_gripper'
    
    #attach in MoveIt scene
    self.scene.attach_box(eef_link, "block_1", touch_links=eef_link)
    #call recommended function
    self.checkObject(10, "block_1", obj_is_attached=True, obj_is_known=True)

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

class scene_object:
  def __init__(self, name="", color="", size=(1,1,1)):
    self.mid_pose = PoseStamped()
    self.color = color
    self.name = name
    self.size = size

