#!/usr/bin/python
from math import sqrt,cos,sin,atan2

class robotPose(object):
  def __init__(self, pose2D, robotID = 0):
    self.pose2D = pose2D
    self.robotID = robotID

class edgeNode(object):
  def __init__(self, node, parent= None, dTheta = None):
    self.node   = node
    self.pNode  = parent
    self.dTheta = dTheta

class edgeList(object):
  def __init__(self, head):
    self.hn = []
    self.hn.append(edgeNode(head))

  def findClosest(self, rand):
    # Finds closest node in the tree to the input rand point
    # -- initialize to first element
    d  = self.distn(self.hn[0].node.pose2D, rand)
    closestNode = self.hn[0]
    # -- go through all the elements
    for item in self.hn :
      newDist = self.distn(item.node.pose2D, rand)
      if newDist < d :
        d  = newDist
        closestNode = item
    return closestNode

  def distn(self,p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))
  
  def appendToNode(self, parent, newNode, dTheta= None, robotID = 0) :
    pose = robotPose(newNode, robotID)
    node = edgeNode(pose, parent, dTheta)
    self.hn.append(node)

  def getPathToRoot(self, queryNodePosition):
    qNode = self.findClosest(queryNodePosition)

    # qNode has the edgeNode object of the node required, from here the list
    # is traversed to the 0th node, which is the first node
    nodeList = []
    while (qNode != self.hn[0]):
      nodeList.append(qNode)
      qNode = qNode.pNode
    nodeList.append(self.hn[0])
    return nodeList
