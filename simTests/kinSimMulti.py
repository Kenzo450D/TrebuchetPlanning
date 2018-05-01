#!/usr/bin/python
##Author: Sayantan Datta Saurav Agarwal 
##E-mail: sayantan.knz@gmail.com sagarw10@uncc.edu 
## The file demonstrates:
##   1. Adding rooms and walls to the environment (refer to buildWorld.py as well)
##   2. Setting up a robot
##   3. Perform collision checking
##   4. Modify the robot configurations and visualize
##   5. Adding text objects and modifying them

import sys
from klampt import *
from klampt import vis
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates
from klampt.math import so3
import klampt.model.collide as collide
import time
import math
import buildWorld as bW
sys.path.append("./kinematics/")
#from sphero6DoF import sphero6DoF
from kobuki import kobuki
from turtlebot import turtlebot
from decimal import Decimal

from rrtMulti import RRTMulti

def collisionDetector(collisionChecker, robot, world, robotID, ttConfigs):
    for rConfig in ttConfigs:
        print "Configuration: ", rConfig
        robot.setConfig(rConfig)
        q = robot.getConfig()
        print 'kinSim.py :: Config: ', q
        collisionFlag = False
        # -- check for robot to terrain collision
        collRT0 = collisionChecker.robotTerrainCollisions(world.robot(robotID), world.terrain(0))
        for i,j in collRT0:
            collisionFlag = True
            strng = "Robot collides with "+j.getName()
            print(strng)
            break

        # -- check for robot to robot collision
        for iR in range(world.numRobots()):
            collRT2 = collisionChecker.robotObjectCollisions(world.robot(iR))
            for i,j in collRT2:
                collisionFlag = True
                strng = world.robot(iR).getName() + " collides with " + j.getName()
                print(strng)
        if (collisionFlag):
            print "\t\t\tCollision!!"
        else:
            print "\t\t\tClear"



def distn(p0, p1):
    d = (p1[0] - p0[0])**2 + (p1[1] - p0[1])**2
    d = math.sqrt(d)
    dth = p1[2] - p0[2]
    #dth = math.atan2((p1[1] - p0[1]), (p1[0] - p0[0]))
    return d, dth

def getOrientation (p0, p1):
    dth = math.atan2((p1[1] - p0[1]), (p1[0] - p0[0]))
    return dth


if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: kinematicSim.py [world_file]"
        exit()

    ## Creates a world and loads all the items on the command line
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    coordinates.setWorldModel(world)

    ## Get walls
    ## Two rooms separated by a wall with a window
    #bW.getDoubleRoomWindow(world, 8, 8, 1.2)
    
    ## Two rooms separated by a wall with a door 
    bW.getDoubleRoomDoor(world, 8, 8, 0.5)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)

    # ==========================================================================
    ## Create robot object. Change the class to the desired robot. 
    ## Also, make sure the robot class corresponds to the robot in simpleWorld.xml file
    #robot = kobuki(world.robot(0), vis)
    #robot.setAltitude(0.01)
    ttStartPos = [-2.5,-2.5,0]
    #ttGoalPos = [-3,1,3.14159]
    ttGoalPos = [-2.5,1.5,0]

    # ==========================================================================
    ## Initialize the robots and their position
    # After declaring a robot, make sure you have added it to the robot List
    # The robot list is sent to the RRT planner to be used for RRT planning.
    # We only send the composite robot convex hulls to the RRT planner

    kobukiL = kobuki(world.robot(0), "kobukiL", vis)
    kobukiL.setAltitude(0.05)
    kobukiS = kobuki(world.robot(1), "kobukiS", vis)
    kobukiS.setAltitude(0.05)
    # robot = turtlebot(world.robot(2), "tbot", vis)
    # robot.setAltitude(0.02)
    # sphero1 = sphero6DoF(world.robot(3), "sphero1", vis)
    # sphere2 = sphero6DoF(world.robot(4), "sphero2", vis)

    # -- add all the robots in robotList
    robotList = []
    robotList.append(kobukiL)
    robotList.append(kobukiS)

    allRobotList = []
    allRobotList.append(kobukiL)
    allRobotList.append(kobukiS)

    # -- robot configurations out of the Map
    noCollisionConfig = []
    noCollisionConfig.append([-10.0,-10.0, 0.0]) # kobuki Large
    noCollisionConfig.append([-60.0,-70.0, 0.0]) # kobuki Small
    noCollisionConfig.append([-50.0,-50.0, 0.0]) # turtleBot
    noCollisionConfig.append([-5.5, -5.5, 0,0,0,0]) #sphere1
    noCollisionConfig.append([-4.5, -4.5, 0,0,0,0]) #sphere2

    # -- set all robots to their no collision configuration
    for index, robot in enumerate(allRobotList) :
        robot.setConfig(noCollisionConfig[index])

    # -- set the turtlebots outside the map
    # tbot2.setConfig([-5,-5,0])
    # sphero1.setConfig([-5,-5,2,0,0,0])
    # sphero2.setConfig([-5, -4.5, 2, 0, 0, 0])

    ## Display the world coordinate system
    vis.add("WCS", [so3.identity(),[0,0,0]])
    vis.setAttribute("WCS", "size", 24)


    #print "Visualization items:"
    #vis.listItems(indent=2)

    #vis.autoFitCamera()
    vis.addText("textCol", "No collision")
    vis.setAttribute("textCol","size",24)
    collisionFlag = False
    collisionChecker = collide.WorldCollider(world)


    ## Trajectory from RRT Code

    robotIDList = [0,1]
    trajPlan = RRTMulti(robotList, robotIDList, noCollisionConfig[0:len(robotList)], world, collisionChecker,  ttStartPos, ttGoalPos, 0)
    ttConfigs, robotConfigs = trajPlan.main()
    print type(ttConfigs[0])
    print "len config: ", len(ttConfigs)
    print "Config[0]: ", ttConfigs[0]
    print "Last Config: ", ttConfigs[-1]
    for idx,item in enumerate(ttConfigs):
        p0 = round(item[0],3)
        p1 = round(item[1],3)
        p2 = round(item[2],3)
        print "kinSim :: Robot :",robotConfigs[idx],"Config: ", idx, " :: ", p0, p1, p2
    print "------------------------------------------------------------"
    
    ## Fix formation of robot to prefer smaller formation for one step after shift
    # This is done so that, the robot doesn't immediately change from smaller to
    # larger formation. Doing so, makes a collision at that time instant
    
    prevConfig = None
    for idx, item in enumerate(robotConfigs) :
        if (idx == 0):
            prevConfig = item
            continue
        if item < prevConfig:
            robotConfigs[idx] = prevConfig
        prevConfig = item
    
    
    
    # collisionChecker, robot, world, robotID, ttConfigs
    # NOTE: Not Required in the final version, this is a unit test
    #collisionDetector(collisionChecker, kobukiL, world, 0, ttConfigs)
    #print "Check Collision with the Smaller Robot! "
    #collisionDetector(collisionChecker, kobukiS, world, 0, ttConfigs)
    #sys.exit(0)
    
    # -- reset the positions of the robots
    for index, robot in enumerate(allRobotList) :
        robot.setConfig(noCollisionConfig[index])
    curRobot = allRobotList[robotConfigs[0]]
    curRobot.setConfig(ttConfigs[0])
    
    ## On-screen text display
    vis.addText("textConfig","Robot configuration: ")
    vis.setAttribute("textConfig","size",24)
    vis.addText("textbottom","WCS: X-axis Red, Y-axis Green, Z-axis Blue",(20,-30))

    print "Starting visualization window#..."

    ## Run the visualizer, which runs in a separate thread
    vis.setWindowTitle("Visualization for kinematic simulation")

    #print(next(collisionFlag))
    vis.show()
    configLen = len(ttConfigs)
    configIdx = 1
    simTime = 600
    startTime = time.time()
    oldTime = startTime
    eps = 0.01
    prevMinTheta = 10 # a random large value
    prevMinDist  = 10 # a random large value
    count = 1
    rTime = None # reached time - measures time after reaching goal
    goalReachFlag = False
    lastConfig = curRobot.getConfig()
    changeConfigurationOfRobot = False
    count = 0

    while vis.shown() and (time.time() - startTime < simTime):
        # if (count == 2000) :
        #     sys.exit(0)
        count = count + 1
        print "Config: ", configIdx, " of ", configLen
        flagOverdone = False
        flagOrientationOverdone = False
        
        # -- Local Visualization
        vis.lock()
        
        # -- Delta Time for current Step
        deltaT = time.time() - oldTime
        oldTime = time.time()
        # print "deltaT: ", deltaT

        # -- check which Robot formation to use
        if configIdx != len(robotConfigs):
            rId = robotConfigs[configIdx]

        print "Last Configuration: ", lastConfig
        print "Robot Configuration to Use: ", rId

        if changeConfigurationOfRobot == True :
            # change the shape of the robot
            # reset the robots
            for index, robot in enumerate(allRobotList) :
                robot.setConfig(noCollisionConfig[index])
            curRobot =  allRobotList[rId]
            curRobot.setConfig(lastConfig)
            changeConfigurationOfRobot = False
            
        
        else :
        # -- update a config in two parts
        # in step 1, update the orientation, to go to next pose
        # in step 2, once in orientation, move towards the position required
        # in a straight line motion(like a turtlebot would)
        # -- get current robot configuration

            q = curRobot.getConfig()
            print ("CurRobot: ", curRobot.robotName)
            print ("CurRobot Position: ", q)

            # -- update the orientation
            # to go to next pose, use get orientation function to check omega
            nodeDxy = 0.0 # initialize
            thetaRequired = 0.0 # initialize
            # if last node, then dont access next node
            if (configIdx == configLen) :
                #nodeDxy, dth = distn(ttConfigs[configIdx-1], ttConfigs[configIdx])
                dxy, dth = distn(q,ttConfigs[configIdx-1])
                nodeDxy = dxy
                thetaRequired = ttConfigs[configIdx-1][2] # come to last orientation
                goalReachFlag = True
            # elif (configIdx == configLen and goalReachFlag) :
            #     # reached goal in previous iteration, now should fix orientation
            #     thetaRequired = dth
            else:
                thetaRequired = getOrientation(ttConfigs[configIdx-1], ttConfigs[configIdx])
                nodeDxy, dth = distn(ttConfigs[configIdx-1], ttConfigs[configIdx])
                dxy, dth = distn(q,ttConfigs[configIdx])
            # -- if current theta is not the theta required, keep changing
            nodeDtheta = thetaRequired - q[2] #q[2] is current orientation

            # fix orientation values outside -pi to +pi range
            if (abs(nodeDtheta) > 3.14159) :
                # if delta theta is more than pi
                if (nodeDtheta < 0):
                    nodeDtheta  = nodeDtheta + 2*3.14159
                else :
                    nodeDtheta = nodeDtheta - 2*3.14159

            # if orientation is not Complete
            if (abs(nodeDtheta) > 0.01) :
                print "Mode: Orientation"
                vel = 0.0
                omega = nodeDtheta * 400
                # print (omega)
                # -- max omega the robot can take is 5
                if omega < 0:
                    omega = -5
                if omega > 0:
                    omega = 5
                deltaT = time.time() - oldTime
                curRobot.velControlKin(vel, omega, deltaT)
                print "Velocity: ", vel
                print "Omega: ", omega
                print "deltaT: ", deltaT
            else:
                # orientation is complete, xy is not complete
                if abs(dxy) > 0.016 :
                    curRobot.setConfig([q[0],q[1],thetaRequired])
                    print "Mode: Velocity"
                    print "d(xy): ", dxy
                    vel = nodeDxy * 100
                    if vel > 5.0:
                        vel = 5.0
                    elif vel < -5.0 :
                        vel = 5.0
                    print "Velocity: ", vel
                    omega = 0.0
                    deltaT = time.time() - oldTime
                    curRobot.velControlKin(vel, omega, deltaT)
                    print "Velocity: ", vel
                    print "Omega: ", omega
                    print "deltaT: ", deltaT
                    # 
                else:
                    # orientation and location is complete
                    print "Mode: Reached"
                    if (goalReachFlag):
                        print "Mode: Goal Position Reached"
                        tmpConfig = [ttConfigs[configIdx-1][0], ttConfigs[configIdx-1][1], thetaRequired]
                    else:
                        tmpConfig = [ttConfigs[configIdx][0], ttConfigs[configIdx][1], thetaRequired]
                    # print "Mode: ", tmpConfig
                    
                    # count = count + 1
                    # if (count == 3):
                    #     sys.exit(0)
                    # print "Mode: Position Reached"
                    if (configIdx == configLen):
                        configIdx = configIdx - 1
                    curRobot.setConfig(tmpConfig)
                    lastConfig = curRobot.getConfig()[0:3]
                    configIdx = configIdx + 1
                    
                    if (configIdx != len(ttConfigs)) and (rId != robotConfigs[configIdx]) :
                        # it is not the goal configuration
                        # the next configuration requires a change of shape
                        changeConfigurationOfRobot = True

        # ======================================================================
        ## Add the visuation text
        # Get the robot configuration
        q = curRobot.getConfig()
        # print 'Robot ',curRobot.robotName,'Config   : ', q
        if configIdx == configLen:
            print "Reached last Config!"
            if (rTime == None) :
                rTime = time.time()
            else:
                nowTime = time.time()
                if (rTime - nowTime > 10):
                    # Stop visualization if its takes way too much time!
                    raise ValueError('ERROR: Time Limit Exceeded!')
                    break
        # else:
            # print 'Required Config: ', ttConfigs[configIdx]
        q2f = [ '{0:.2f}'.format(elem) for elem in q]
        strng = "Robot configuration: " + str(q2f)
        vis.addText("textConfig", strng)

        # ======================================================================
        ## Checking collision
        collisionFlag = False
        #for i,j in collisionChecker.collisionTests():
        #    if i[1].collides(j[1]):
        #        collisionFlag = True
        #        strng = "Object "+i[0].getName()+" collides with "+j[0].getName()
        #        print(strng)
        #        vis.addText("textCol", strng)
        #        vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
        collRT0 = collisionChecker.robotTerrainCollisions(world.robot(robotConfigs[configIdx-1]), world.terrain(0))
        for i,j in collRT0:
            collisionFlag = True
            strng = "Robot collides with "+j.getName()
            print(strng)
            vis.addText("textCol", strng)
            vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
            break
  
        # robot object collision gives an (i,j) for the place where the collision has taken place
        for iR in range(world.numRobots()):
            collRT2 = collisionChecker.robotObjectCollisions(world.robot(iR))
            for i,j in collRT2:
                collisionFlag = True
                strng = world.robot(iR).getName() + " collides with " + j.getName()
                print(strng)
                vis.addText("textCol", strng)
                vis.setColor("textCol", 0.8500, 0.3250, 0.0980)

        # -- self collission
        #collRT3 = collisionChecker.robotSelfCollisions()
        #for i,j in collRT3:
            #collisionFlag = True
            #strng = i.getName() + " collides with "+j.getName()
            #print(strng)
            #vis.addText("textCol", strng)
            #vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
        if not collisionFlag:
            vis.addText("textCol", "No collision")
            vis.setColor("textCol", 0.4660, 0.6740, 0.1880)

        if not collisionFlag:
            vis.addText("textCol", "No collision")
            vis.setColor("textCol", 0.4660, 0.6740, 0.1880)

        vis.unlock()
        #changes to the visualization must be done outside the lock
        # time.sleep(0.01)
    vis.clearText()

    print "Ending klampt.vis visualization."
    vis.kill()
