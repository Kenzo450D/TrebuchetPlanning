#!/usr/bin/env python

# rrt.py
# This program generates a simple rapidly
# exploring random tree (RRT) in a rectangular region.

import sys, random, math, pygame, time
from pygame.locals import *
from math import sqrt,cos,sin,atan2

# -- klampt imports
from klampt import *
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide

XDIM = 800
YDIM = 800
EPSILON = 10
ENDEPSILON = 1
NUMNODES = 50000
PIBY18 = 0.174533
NODETHRESHOLD = 2

from multiEdgeList import robotPose
from multiEdgeList import edgeNode
from multiEdgeList import edgeList

class RRTMulti:
  def __init__(self, robotList, robotIDList, robotOutWSNoCollisionSetup, world, collisionChecker, startPos= None, endPos= None, startRobotID = 0, goalRobotID = 0, epsilon = None, debug = 0):
    # -- RobotList object
    # RobotList is a list of robots from the largest to the smallest size.
    # The robot list should only contain larger than usual robots,
    # which should emulate an entire configuration.
    # The first robot in the robot list should be the largest configuration
    self.robotList = robotList
    if (debug == 1) :
      print "Robot list sent to RRT:"
      for idx, item in enumerate(robotList):
        print "Robot sent[",idx, "]     :", item

      for idx, item in enumerate(self.robotList):
        print "Robot saved[",idx,"]     :", item

    # -- robotIDList
    # Stores the ID of the robot as stored in the parent function, this is
    # required for the collision detection step
    self.robotIDList = robotIDList

    # -- robotOutWSNoCollisionSetup list
    # robotOutWSNoCollisionSetup is the default setup of the robot when
    # they are out of the workspace, and they are not in any collision config
    # The idea to use this is cause, when we are trying out different robot
    # configurations, we want to make sure all the other robots are out of the
    # workspace, and then just try one robot at a time

    self.robotNCSetup = robotOutWSNoCollisionSetup
    if (debug == 1) :
      for idx, item in enumerate(self.robotNCSetup):
        print "Robot No Collission Setup [",idx,"]: ", item

    # -- World and collisionChecker
    self.world = world
    if (debug == 1):
      print "World Sent to it:", world
      print "World saved     :", self.world

    self.collisionChecker = collisionChecker
    if debug == 1:
      print "CC Sent to it:", collisionChecker
      print "CC saved     :", self.collisionChecker


    # -- Start Position
    if (startPos == None):
      self.startPos = robotPose([XDIM/4.0,YDIM/4.0, 0], startRobotID)
    else:
      scaledStartPos = self.scaleKlampt2DPoseToRRT2DPose(startPos)
      self.startPos = robotPose(scaledStartPos, startRobotID)

    # -- End Position
    if (endPos == None):
      self.goalPos  = robotPose([3.0*XDIM/4.0, 3.0*YDIM/4.0, 0],goalRobotID)

    else:
      scaledEndPos = self.scaleKlampt2DPoseToRRT2DPose(endPos)
      self.goalPos  = robotPose(scaledEndPos, goalRobotID)
    
    if (debug == 1) :
      print ("Start Position in RRT: "), self.startPos.pose2D
      print ("Goal Position in RRT:  "), self.goalPos.pose2D
      # -- Robot ID
      print "Start Robot ID         :", self.startPos.robotID
      print "Goal  Robot ID         :", self.goalPos.robotID

    # -- Epsilon
    if (epsilon == None):
      self.EPSILON = EPSILON
    else:
      self.EPSILON = epsilon

    print "Epsilon : ", self.EPSILON
        
# ------------------------------------------------------------------------------
# Function: Dist
# Calculate distance between two points
# ------------------------------------------------------------------------------
  def dist(self,p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

# ------------------------------------------------------------------------------
# Function: findClosestPoint
# Find the closest Point from current set of vertices towards sampled point
# ------------------------------------------------------------------------------
  def findClosestPoint(self, nodes, rand):
    # rand is a randomly position in the state space, the goal is to find the
    # node in nodes which is closest to rand, and then append to that node
    # the new node, and also the control statement to make it work
    if nodes == None:
      return None

    clNode = nodes.findClosest(rand)
    return clNode


# ------------------------------------------------------------------------------
# Function: collisionCheck
# Checks if robot configuration is in collision with env
# True if there no collision
# False if there is collision
# ------------------------------------------------------------------------------
  def collisionCheck(self, node):
    # -- Set the robot configuration
    # node is a 3 element list with the x, y and theta information
    robotIdx = 0
    while (robotIdx < len(self.robotList)):
      # print "Collision Detector :: Robot Config: ", node
      # print "Collision Detector :: Robot ID    : ", robotIdx
      
      # -- make sure all robots are kicked out of the map
      for idx,robot in enumerate(self.robotList):
        robot.setConfig(self.robotNCSetup[idx])

      # -- put the robot in the configuration
      # get the robot required to tmpRobot
      tmpRobot = self.robotList[robotIdx]
      
      # -- scale the test node to klampt workspace
      # this is done to make sure the my workspace which is [0:800]x[0:800] can
      # scale down to the klampt workspace which is [-4 : 4]x[-4 : 4]
      sNode = self.scaleNode(node)
      tmpRobot.setConfig(sNode)
      q = tmpRobot.getConfig()
      # -- check if there is collision on the current robot
      
      # print "rrt.py :: q : ", q 
      # -- check if config is collision free
      collisionFlag = False

      # No checks made for robot

      # -- check for robot to terrain collision
      collRT0 = self.collisionChecker.robotTerrainCollisions(self.world.robot(self.robotIDList[robotIdx]), self.world.terrain(0))
      for i,j in collRT0:
        collisionFlag = True
        # strng = "Robot collides with "+j.getName()
        # print(strng)
        break

      # -- check for robot to robot collision
      for iR in range(self.world.numRobots()):
        collRT2 = self.collisionChecker.robotObjectCollisions(self.world.robot(iR))
        for i,j in collRT2:
          collisionFlag = True
          # strng = self.world.robot(iR).getName() + " collides with " + j.getName()
          # print(strng)

      if collisionFlag == False:
        # there is no collission
        # if robotIdx == 1:
          # print "Collission Check : ", robotIdx, "Sucess!"
        return True, self.robotIDList[robotIdx]
      else :
        # print "Collission Check : ", robotIdx, "Failed!"
        # if (robotIdx == 1):
          # print "Collission Check : ", robotIdx, "Failed!"
        robotIdx = robotIdx + 1
    # END: While loop :: while (robotIdx < len(self.robotList)):

    # -- Return false if no robot can make it through
    return False, None
  



# ------------------------------------------------------------------------------
# Function: move_from_to
# Find the movement step, calculate next step towards sample node (rand)
# ------------------------------------------------------------------------------
  def move_from_to(self, nodes, clNode, rand):
    # nodes: set of nodes calculated
    # clNode: closest node towards the sampled point
    # rand: sampled point

    # -- if distance between two points is within one hop
    if self.dist(clNode.node.pose2D, rand) < self.EPSILON:
      theta = atan2(rand[1]-clNode.node.pose2D[1],rand[0]-clNode.node.pose2D[0])
      rand.append(theta)
      colCheck, robotID = self.collisionCheck(rand)
      if (colCheck):
        nodes.appendToNode(clNode, rand, (theta - clNode.node.pose2D[2]), robotID)
        return rand
      else:
        return None

    # -- if distance between two points is not within one hop
    else:
      theta = atan2(rand[1]-clNode.node.pose2D[1],rand[0]-clNode.node.pose2D[0])
      # -- limit the maximum turn they can take to 10 degrees
      dTheta = theta - clNode.node.pose2D[2]
      if (dTheta > PIBY18) :
        dTheta = PIBY18
      elif (dTheta < -PIBY18) :
        dTheta = -PIBY18
      orientation = clNode.node.pose2D[2] + dTheta
      # newPosition = [clNode.node[0] + self.EPSILON*cos(theta), clNode.node[1] + EPSILON*sin(theta), theta]
      newPos_x = clNode.node.pose2D[0] + EPSILON*cos(orientation)
      newPos_y = clNode.node.pose2D[1] + EPSILON*sin(orientation)
      newPos_th = orientation
      newPosition = [newPos_x, newPos_y, newPos_th]
      # -- check if node is collision free or not
      colCheck, robotID = self.collisionCheck(newPosition)
      #print ("Collision Check Result: ", colCheck)
      #print ("RobotID used: ", robotID)
      if (colCheck == True):
        # collision Free Setup
        nodes.appendToNode(clNode, newPosition, (theta - clNode.node.pose2D[2]), robotID)
        return newPosition
      else :
        # collision configuration
        return None

  def move_from_to_gNodes(self, nodes, clNode, rand):
    # nodes: set of nodes calculated
    # clNode : closest node towards the sample point

    # -- if distance between two points is within one hop
    if self.dist(clNode.node.pose2D, rand) < self.EPSILON:
      theta = atan2(rand[1]-clNode.node.pose2D[1],rand[0]-clNode.node.pose2D[0])
      rand.append(theta)
      colCheck, robotID = self.collisionCheck(rand)
      if (colCheck):
        nodes.appendToNode(clNode, rand, (theta - clNode.node.pose2D[2]), robotID)
        return rand
      else:
        return None

    # -- if distance between two points is not within one hop
    else:
      theta = atan2(rand[1]-clNode.node.pose2D[1],rand[0]-clNode.node.pose2D[0])
      # -- limit the maximum turn they can take to 10 degrees
      dTheta = theta - clNode.node.pose2D[2]
      if (dTheta > PIBY18) :
        dTheta = PIBY18
      elif (dTheta < -PIBY18) :
        dTheta = -PIBY18
      orientation = clNode.node.pose2D[2] + dTheta
      newPos_x = clNode.node.pose2D[0] + EPSILON*cos(orientation)
      newPos_y = clNode.node.pose2D[1] + EPSILON*sin(orientation)
      newPos_th = orientation
      # newPosition = [clNode.node[0] + self.EPSILON*cos(theta), clNode.node[1] + EPSILON*sin(theta), theta]
      newPosition = [newPos_x, newPos_y, newPos_th]
      # -- check if node is collision free or not
      colCheck, robotID = self.collisionCheck(newPosition)
      if (colCheck == True):
        # collision Free Setup
        nodes.appendToNode(clNode, newPosition, (theta - clNode.node.pose2D[2]), robotID)
        return newPosition
      else :
        # collision configuration
        return None


# ------------------------------------------------------------------------------
# Function: scalePath
# Scales a list of nodes to be used by klampt. Also converts it to a list of 
# [x,y,theta] rather than a edgeNode object
# ------------------------------------------------------------------------------
  def scalePath(self, nodepath):
    scaledPath = []
    for nodes in nodepath:
      scNode = []
      scNode.append((nodes.node.pose2D[0] - XDIM/2.0)/100)
      scNode.append((nodes.node.pose2D[1] - YDIM/2.0)/100)
      scNode.append(nodes.node.pose2D[2])
      scaledPath.append(scNode)
    return scaledPath

  def checkScaledPath(self, nodepath):
    for nodes in nodepath:
      x = nodes[0]
      y = nodes[1]
      if (x < -4 or x > 4):
        return False
      if (y < -4 or y > 4):
        return False
      return True

  def scaleNode (self, node):
    sNode = []
    sNode.append((node[0] - XDIM/2.0)/100)
    sNode.append((node[1] - YDIM/2.0)/100)
    sNode.append(node[2])
    return sNode

  def scaleKlampt2DPoseToRRT2DPose(self, node) :
    # Here the 2D pose is just a list for (x,y,theta). It does not consider
    # robot ID
    sNode = []
    sNode.append(node[0]*100 + XDIM/2.0)
    sNode.append(node[1]*100 + YDIM/2.0)
    sNode.append(node[2])
    return sNode

# ------------------------------------------------------------------------------
# Function: scalePath
# Scales a list of nodes to be used by klampt. Also converts it to a list of 
# [x,y,theta] rather than a edgeNode object
# ------------------------------------------------------------------------------
  def getRobotIDFromPath(self, totalPath):
    rIdPath = []
    for nodes in totalPath:
      rIdPath.append(nodes.node.robotID)
    return rIdPath

# ------------------------------------------------------------------------------
# Function: reverseGoalPathTheta
# Reverses the orientation of goal path by 180 degrees
# ------------------------------------------------------------------------------
  def reverseGoalPathTheta (self, nodepath) :
    for idx,nodes in enumerate(nodepath):
      # tmpTh is temporary Theta to hold orientation
      tmpTh = nodes.node.pose2D[2] + 3.14159
      oldTmpTh = tmpTh
      # make sure orientation is between -pi and +pi
      if (tmpTh > 3.14159) : 
        # if orientation is more than pi, subtract 2pi from it
        tmpTh = tmpTh - 6.28318
        # if orientation is less than -pi, add 2pi to it
      if (tmpTh < -3.14159) :
        tmpTh = tmpTh + 6.28318
      #print "oldTmpTh: ", oldTmpTh, "---New: ", tmpTh
      nodes.node.pose2D[2] = nodes.node.pose2D[2] + 3.14159
      nodepath[idx].node.pose2D[2] = tmpTh


# ------------------------------------------------------------------------------
# Function: printPath
# Prints the entire path out to console
# ------------------------------------------------------------------------------
  def printPath(self, nodepath):
    for nodes in nodepath:
      nInfo = nodes.node.pose2D
      print (nInfo)

# ------------------------------------------------------------------------------
# Function: main
# RRT implementation without collision detection
# ------------------------------------------------------------------------------
  def main(self):
    # -- initialize start and goal position
    # by default we consider that the start and goal position considers the
    # robot 0 of the two configurations
    start = self.startPos
    goal  = self.goalPos

    sNodes = edgeList(start)
    gNodes = edgeList(goal)
    
    lastNodeStart = None
    lastNodeGoal = None

    orderIsNormal = True # check if start is start and goal is goal or swapped
    # print "Num-nodes: ", NUMNODES
    for i in range(NUMNODES):
      # -- find random position
      rand = [random.random()*float(XDIM), random.random()*float(YDIM)]
      # print "Iteration : ", i
      #if (i % 1000 == 0) :
        #print "Nodes ... ", i
      # -- start with first node to search for closest node
      nn = self.findClosestPoint(sNodes, rand)
      # -- find step from closest node towards random position
      if orderIsNormal == True :
        # sNodes has start Node
        newnode = self.move_from_to(sNodes, nn, rand)
      else:
        # sNodes has end Node
        newnode = self.move_from_to_gNodes(sNodes, nn,rand)
      if newnode == None :
        # node is not feasible:
        continue

      # -- node is feasible
      newNodeOld = newnode

      ## CALCULATE POSITION FOR THE OTHER NODE
      # -- start with the first node to search for closest node
      # print "Node Found :", newNodeOld
      # print "Rand for first path: ", rand
      rand = newnode
      # print "Rand for second path: ", rand
      nn = self.findClosestPoint(gNodes, rand[0:2])
      # print "Closest node : ", nn
      # -- find the step from closest node towards position of start Node
      if orderIsNormal == True:
        # gNodes has the end Node
        newnode = self.move_from_to_gNodes(gNodes, nn, rand[0:2])
      else:
        # gNodes has the start Node
        newnode = self.move_from_to(gNodes, nn, rand[0:2])
      
      #print "New Node calculated: ", newnode

      if newnode == None :
        # node is not feasible:
        newnode = nn.node.pose2D

      #print "Final New Node decided: ", newnode

      # -- CHECK DISTANCE BETWEEN THE TWO CREATED NODES

      # -- print distance:
      #print "distance between the two new nodes: ", self.dist(newNodeOld, newnode)

      # -- if distance is less than threshold
      if (self.dist(newNodeOld, newnode) < NODETHRESHOLD) :
        # time.sleep(10)
        # print "They Connected!"
        # print "Connection Point = ", newNodeOld
        # print "From other side  = ", newnode
        if (sNodes.hn[0].node.pose2D == start.pose2D):
          lastNodeStart = newNodeOld
          lastNodeGoal = newnode
          # print "Last Node Start: ", lastNodeStart
          # print "Last Node Goal: ", lastNodeGoal
        elif (gNodes.hn[0].node.pose2D == start.pose2D) :
          lastNodeGoal = newNodeOld
          lastNodeStart = newnode
          # print "Last Node Goal: ", newNodeOld
          # print "Last Node Start:", newnode
        break
      # print ("----------------------------------------------------------------")
      #print i, "    ", nodes
      # -- swap the  start and goal nodes
      # ---- Swap the order tag
      if orderIsNormal:
        orderIsNormal = False
      else:
        orderIsNormal = True
      # ---- Swap the start and goal nodes
      sNodes, gNodes = gNodes, sNodes
    # END: For loop


    # -- print the set of nodes required
    # Check if RRT path is found or not
    if (lastNodeStart == None or lastNodeGoal == None) :
        print "RRT Path not found!"
        exit(0)


    ## GET PATH
    startPath = sNodes.getPathToRoot(lastNodeStart)
    startPath.reverse()
    goalPath = gNodes.getPathToRoot(lastNodeGoal)


    # -- Test print the paths
    # print "Start Path: "
    # self.printPath(startPath)
    # print "Goal Path: "
    # self.printPath(goalPath)

    # print "Start Path Node 0: ", startPath[0].node.pose2D
    # print "Goal Position: ", self.goalPos
    # print "Start Path Node -1: ", startPath[-1].node.pose2D

    # -- make reverse of scaled path if required
    if (startPath[0].node.pose2D == self.goalPos.pose2D):
      startPath, goalPath = goalPath, startPath
      goalPath.reverse()
      startPath.reverse()
      # print ("Swapped start and goal path!")
      # print ("Start Nodes: ")
      # self.printPath(startPath)
      # print ("Goal Nodes: ")
      # self.printPath(goalPath)

    # -- reverse theta in goal path
    # this is done so that the robot is oriented correctly while coming to 
    # goal position
    self.reverseGoalPathTheta(goalPath)

    totalPath = startPath + goalPath

    # # -- make reverse of scaled path if required
    # if (totalPath[0].node.pose2D == self.goalPos):
    #   totalPath.reverse()
    
    # -- Scale path for klampt
    scaledPath = self.scalePath(totalPath)

    robotIDInPath = self.getRobotIDFromPath(totalPath)

    chkScaledPath = self.checkScaledPath(scaledPath)
    if (chkScaledPath) : 
      print "Scaled Path works!"
    else:
      print "Scaled Path is Faulty!, fix code!"
      sys.exit(0)

    # -- print the scaled path
    #for idx,item in enumerate(scaledPath):
      #print "Config: ", idx, " :: ", item

    # print(scaledPath)
    return scaledPath, robotIDInPath