#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION

class kmr19RobotCtrl:
  def __init__(self):
    self.rs = baxter_interface.RobotEnable(CHECK_VERSION)

  def enableRobot(self):
    self.rs.enable()

  def disableRobot(self):
    self.rs.disable()
