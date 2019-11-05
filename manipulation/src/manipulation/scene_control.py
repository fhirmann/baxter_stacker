#!/usr/bin/env python

import moveit_commander
from geometry_msgs.msg import PoseStamped

class kmr19SceneControl:
  def __init__(self):
    self.scene = moveit_commander.PlanningSceneInterface()

  def addFixedSceneObjects(self):
    #create table
    p = PoseStamped()
    p.header.frame_id = "/base"
    p.pose.position.x = 0.93
    p.pose.position.y = 0.3
    p.pose.position.z = -0.55
    #add table to scene
    self.scene.add_box( "table", p, (0.8, 1.6, 0.7) )

    #create fixed block for demonstration
    p.pose.position.x = 0.82
    p.pose.position.y = 0.065
    p.pose.position.z = -0.16
    #add block
    self.scene.add_box("yellow_block", p, (0.04, 0.04, 0.08))

    print("added fixed scene objects!")
