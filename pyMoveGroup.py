#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from leap_motion.msg import leapros
from leap_motion.msg import Human
from leap_motion.msg import Hand
from math import pi
import Tkinter as tk
import time

def all_close(goal, actual, tolerance):
  all_equal = True

  if type(goal) is list:
    for i in range(len(goal)):
      if abs(actual[i] - goal[i])>tolerance:
        return False

  if type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class LeapMoveGroup(object):

  def __init__(self):
    super(LeapMoveGroup, self).__init__()
    #Initialise commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    print "Initialising Node.."
    rospy.init_node('leapMoveIt', anonymous=True)
    rospy.Rate(50)
    print "Node Initialised"
    #Instantiate outer-level robot interface
    robot = moveit_commander.RobotCommander()
    #Instantiate surrounding world interface
    scene = moveit_commander.PlanningSceneInterface()
    #Panda interface
    group = moveit_commander.MoveGroupCommander("panda_arm")
    handGroup = moveit_commander.MoveGroupCommander("panda_arm_hand")

    self.subscriber = None
    self.robot = robot
    self.scene = scene
    self.group = group #Arm group
    self.handGroup = handGroup #Hand/gripper group

    #GUI Main Window
    self.gui_mainWindow = tk.Tk()
    self.gui_mainWindow.title("LEAP ROS | Control Panel")
    self.gui_mainWindow.geometry("600x300")
    self.gui_mainWindow.grid_rowconfigure(0, weight=1)
    self.gui_mainWindow.grid_columnconfigure(0, weight=1)
    self.gui_mainWindow.resizable(False, True)

    #GUI Current Position
    self.posText = tk.StringVar()
    self.posLabel = tk.Label(self.gui_mainWindow, textvariable=self.posText).grid(row=0, column=0)
    self.posText.set("Current Position\n\nN/A.\n")

    #GUI Previous Position
    self.prevText = tk.StringVar()
    self.prevLabel = tk.Label(self.gui_mainWindow, textvariable=self.prevText).grid(row=1, column=0)
    self.prevText.set("Previous Position\n\nN/A.\n")

    #GUI Reset Button
    self.resetButton = tk.Button(self.gui_mainWindow, text="Reset Position", command=self.resetRobotPos, width=20)
    self.resetButton.configure(bg="red")
    self.resetButton.grid(row=2, column=0)
    
    #GUI Quit Button
    self.quitButton = tk.Button(self.gui_mainWindow, text="Quit", command=self.quitApplication, width=20)
    self.quitButton.configure(bg="red")
    self.quitButton.grid(row=3, column=0)

    self.dpRound = 1 #3 Decimal Points for Rounding
    #Init previous position variables
    self.prevPosx = 0.0
    self.prevPosy = 0.0
    self.prevPosz = 0.0
    self.idling = False

    #Subscribed to leap data
    self.subscriber = rospy.Subscriber("/leap_motion/leap_filtered", Human, self.callback, queue_size=1, buff_size=52428800)
    print "Subscribed."

    #Initial robot position is saved
    self.initialPos = geometry_msgs.msg.Pose().position
    self.initialPos.x = group.get_current_pose().pose.position.x
    self.initialPos.y = group.get_current_pose().pose.position.z
    self.initialPos.z = group.get_current_pose().pose.position.y
    self.initialOri = group.get_current_pose().pose.orientation
  
    self.idling = True #Variable used to stop callback running gotopose
    self.gui_mainWindow.mainloop()

  def callback(self, msg):
    print('idling=', self.idling)
    self.rightHand = msg.right_hand
    self.leftHand = msg.left_hand

    #Assign pinch pose variable (distance between left hand index and thumb)
    self.pinchStr = self.leftHand.pinch_strength
    
    #Assign palm position variables
    self.palmRight = self.rightHand.palm_center
    self.palmLeft = self.leftHand.palm_center
    
    if self.idling:
      self.idling = False
      print "callback"
      #Check that right hand leap data is not 0
      if self.palmRight.x != 0 or self.palmRight.y != 0 or self.palmRight.z != 0:
        print "running go to pose"
        self.go_to_pose_goal()
      #Check that left hand leap data is not 0
      if self.palmLeft.x != 0:
        print "running hand goal"
        self.hand_goal()
      self.idling = True

    #right hand stuff
    #posRightData = [rightHand.roll, rightHand.pitch, rightHand.yaw]
    #self.posRightPalm = [self.palmRight.x, self.palmRight.y, self.palmRight.z]

    #left hand stuff
    #posLeftData = [leftHand.roll, leftHand.pitch, leftHand.yaw]
    #posLeftPalm = [palmLeft.x, palmLeft.y, palmLeft.z]

    #rospy.loginfo('\nRight Palm: {}'.format(self.posRightPalm))

  #Euler angles to quaternion
  def euler_to_quater(self, roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return qw, qx, qy, qz

  def go_to_pose_goal(self):
    group = self.group
    group.stop()
    pose_goal = geometry_msgs.msg.Pose()
    #Update GUI on current robot position
    self.posText.set("Current Position\nx: %.3f\ny: %.3f\nz: %.3f" % (self.rightHand.roll, self.rightHand.pitch, self.rightHand.yaw))
    #Convert hand euler to quaternion
    self.orW, self.orX, self.orY, self.orZ = self.euler_to_quater(self.rightHand.roll, self.rightHand.pitch, self.rightHand.yaw)
    print self.orW, self.palmRight.x, self.palmRight.y, self.palmRight.z
    #pose_goal.orientation.x = self.orX
    #pose_goal.orientation.y = self.orY
    #pose_goal.orientation.z = self.orZ
    #pose_goal.orientation.w = 1.0
    pose_goal.position.x = self.palmRight.x
    pose_goal.position.y = -1 * self.palmRight.z #Data inverted
    pose_goal.position.z = self.palmRight.y #Z & Y are swapped as LEAP is facing upwards
    pose_goal.orientation = self.initialOri
    
    #Desired Positions are set for checking
    self.desiredPosx = pose_goal.position.x
    self.desiredPosy = pose_goal.position.y
    self.desiredPosz = pose_goal.position.z
    
    #Positions are rounded to make checking easier
    self.prevPosx = round(self.prevPosx, self.dpRound)
    self.desiredPosx = round(self.desiredPosx, self.dpRound)
    self.prevPosy = round(self.prevPosy, self.dpRound)
    self.desiredPosy = round(self.desiredPosy, self.dpRound)
    self.prevPosz = round(self.prevPosz, self.dpRound)
    self.desiredPosz = round(self.desiredPosz, self.dpRound)

    print "desired = ", self.desiredPosx
    print "previous = ", self.prevPosx

    #Check if desired position does not match previous position
    if self.prevPosx != self.desiredPosx or self.prevPosy != self.desiredPosy or self.prevPosz != self.desiredPosz:
      print "Moving..."
      #Accepted positions are set as previous for next run
      self.prevPosx = self.desiredPosx
      self.prevPosy = self.desiredPosy
      self.prevPosz = self.desiredPosz
      self.prevText.set("Previous Position\nx: %.3f\ny: %.3f\nz: %.3f" % (self.prevPosx, self.prevPosy, self.prevPosz)) #Set previous GUI text
      
      #Execute Movement
      group.set_pose_target(pose_goal)
      group.go(wait=True)
      group.stop()
      group.clear_pose_targets() #Clear all targets
      print "Done"

      current_pose = self.group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 0.01)

    else:
      print "Desired position is too similar to previous position...."
    
  def hand_goal(self):
    handGroup = self.handGroup
    print "running hand goal..."

    hand_joint_goal = handGroup.get_current_joint_values()
    #Fingers (7 & 8)
    hand_joint_goal[7] = 0.035 - (self.pinchStr*0.035) #pinchStr = dist between index and thumb (left hand)
    hand_joint_goal[8] = 0.035 - (self.pinchStr*0.035)

    #Execute Movement
    handGroup.go(hand_joint_goal, wait=True)
    handGroup.stop()
    handGroup.clear_pose_targets()
    current_hand_pose = handGroup.get_current_joint_values()
  
  #Function for GUI reset button
  def resetRobotPos(self):
    if self.idling:
      print "Resetting Robot Position..."
      group = self.group
      reset_goal = geometry_msgs.msg.Pose()
      #Set goals to original/initial position
      reset_goal.position.x = self.initialPos.x
      reset_goal.position.y = self.initialPos.z
      reset_goal.position.z = self.initialPos.y
      reset_goal.orientation = self.initialOri

      #Execute Movement
      group.set_pose_target(reset_goal)
      plan = group.go(wait=True)
      group.stop()
      group.clear_pose_targets()
      print "Robot Reset"

  def quitApplication(self):
    sys.exit()

def main():
  try:
    leaps = LeapMoveGroup()
    raw_input()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
