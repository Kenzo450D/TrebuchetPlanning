#!/usr/bin/python
##Author: Saurav Agarwal
##E-mail: sagarw10@uncc.edu
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
from sphero6DoF import sphero6DoF
#from kobuki import kobuki
from turtlebot import turtlebot
from decimal import Decimal

from rrt import RRT

def collisionDetector(collisionChecker, world, robotID, ttConfigs):
    for rConfig in ttConfigs:
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
    bW.getDoubleRoomDoor(world, 8, 8, 0.8)

    ## Add the world to the visualizer
    vis.add("world",world)

    vp = vis.getViewport()
    vp.w,vp.h = 1200,800
    vis.setViewport(vp)

    ## Create robot object. Change the class to the desired robot. 
    ## Also, make sure the robot class corresponds to the robot in simpleWorld.xml file
    #robot = kobuki(world.robot(0), vis)
    #robot.setAltitude(0.01)
    ttStartPos = [-3,-1,0]
    #ttGoalPos = [-3,1,3.14159]
    ttGoalPos = [-3,1,0]
    robot = turtlebot(world.robot(0), "turtle", vis)
    robot.setAltitude(0.02)
    # print type(ttConfigs[0])
    # print "len config: ", len(ttConfigs)
    # print "Config[0]: ", ttConfigs[0]
    # print "Last Config: ", ttConfigs[-1]
    # for idx,item in enumerate(ttConfigs):
    #     print "kinSim :: Config: ", idx, " :: ", item
    # print "------------------------------------------------------------"
    # # sys.exit(0)
    # robot.setConfig(ttConfigs[0])
    robot.setConfig(ttStartPos)
    
    # robotSphere = sphero6DoF(world.robot(1), "sphero", vis)
    # robotSphere.setConfig([0,0,2,0,0,0])

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
    trajPlan = RRT(robot, world, collisionChecker,  ttStartPos, ttGoalPos, 0)
    ttConfigs = trajPlan.main()
    print type(ttConfigs[0])
    print "len config: ", len(ttConfigs)
    print "Config[0]: ", ttConfigs[0]
    print "Last Config: ", ttConfigs[-1]
    for idx,item in enumerate(ttConfigs):
        print "kinSim :: Config: ", idx, " :: ", item
    print "------------------------------------------------------------"
    collisionDetector(collisionChecker, world, 0, ttConfigs)
    # sys.exit(0)
    robot.setConfig(ttConfigs[0])

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
    while vis.shown() and (time.time() - startTime < simTime):
        
        print "Config: ", configIdx, " of ", configLen
        flagOverdone = False
        flagOrientationOverdone = False
        vis.lock()
        deltaT = time.time() - oldTime
        oldTime = time.time()
        # print "deltaT: ", deltaT
        # -- update a config in two parts
        # in step 1, update the orientation, to go to next pose
        # in step 2, once in orientation, move towards the position required
        # in a straight line motion(like a turtlebot would)
        # -- get current robot configuration
        q = robot.getConfig()
        # -- update the orientation
        # to go to next pose, use get orientation function to check omega
        nodeDxy = 0.0 # initialize
        thetaRequired = 0.0 # initialize
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
        if (abs(nodeDtheta) > 3.14159) :
            # if delta theta is more than pi
            if (nodeDtheta < 0):
                nodeDtheta  = nodeDtheta + 2*3.14159
            else :
                nodeDtheta = nodeDtheta - 2*3.14159
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
            robot.velControlKin(vel, omega, deltaT)
        else:
            if abs(dxy) > 0.016 :
                robot.setConfig([q[0],q[1],thetaRequired])
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
                robot.velControlKin(vel, omega, deltaT)
                # 
            else:
                print "Mode: Reached"
                if (goalReachFlag):
                    print "Mode: Goal Position Reached"
                    tmpConfig = [ttConfigs[configIdx-1][0], ttConfigs[configIdx-1][1], thetaRequired]
                else:
                    tmpConfig = [ttConfigs[configIdx][0], ttConfigs[configIdx][1], thetaRequired]
                print "Mode: ", tmpConfig
                
                # count = count + 1
                # if (count == 3):
                #     sys.exit(0)
                print "Mode: Position Reached"
                if (configIdx == configLen):
                    configIdx = configIdx - 1
                robot.setConfig(tmpConfig)
                configIdx = configIdx + 1
        """


        # ----------------------------------------------------------------------
        # OLD IMPLEMENTATION OF CONFIGURATION :: START

        print "Config: ", configIdx, " of ", configLen
        flagOverdone = False
        flagOrientationOverdone = False
        vis.lock()
        deltaT = time.time() - oldTime
        oldTime = time.time()
        # print "deltaT: ", deltaT
        # -- update a config in two parts
        # in step 1, update the orientation, and in the next step update
        # the position. To update the orientation, take a very small omega
        # and then move it to that orientation required
        # in step 2, once in orientation, move towards the position required
        # in a straight line motion(like a turtlebot would)
        # -- update the orientation
        q = robot.getConfig()
        nodeDxy, nodeDtheta = distn(ttConfigs[configIdx - 1],ttConfigs[configIdx])
        dxy, dth = distn(q,ttConfigs[configIdx])
        print "deltaxy: ", dxy
        print "deltath: ", dth
        # check when the distances are overdone (the robot is crossing pos)
        if abs(dxy) < prevMinDist:
            prevMinDist = abs(dxy)
        elif abs(dxy) > prevMinDist:
            flagOverdone = True

        # # check when the orientation is overdone
        # if abs(dth) < prevMinTheta :
        #     prevMinTheta = abs(dth)
        # elif abs(dth) > prevMinTheta:
        #     flagOrientationOverdone = True

        # -- check if position has been reached
        if ((abs(dxy) < 0.01 or flagOverdone) and (abs(dth) < 0.01) or flagOrientationOverdone) : 
            print "Mode: Position Reached"
            if (configIdx == configLen):
                configIdx = configIdx - 1
            robot.setConfig(ttConfigs[configIdx])
            configIdx = configIdx + 1
            prevMinTheta = 10
            prevMinDist = 10
            flagOverdone = False
            flagOrientationOverdone = False
        elif abs(dth) > 0.01 or flagOrientationOverdone:
            print "Mode: Omega Control"
            # change orientation of robot
            vel = 0.0
            omega = nodeDtheta * 400
            # print (omega)
            # -- max omega the robot can take is 1
            if omega < -5:
                omega = -5
            if omega > 5:
                omega = 5
            deltaT = time.time() - oldTime
            robot.velControlKin(vel, omega, deltaT)

        # elif (abs(dxy) > 0.01 and abs(dth) < 0.01) :
        #     print "Mode: Velocity Control"
        #     # change the location of robot
        #     omega = 0.0
        #     vel = nodeDxy * 100
        #     # -- max omega the robot can take is 0.2
        #     if vel < - 5:
        #         vel = -5
        #     if vel > 5 :
        #         vel = 5
        #     deltaT = time.time() - oldTime
        #     robot.velControlKin(vel, omega, deltaT)
        # OLD IMPLEMENTATION OF CONFIGURATION :: END
        # ----------------------------------------------------------------------
        # else :


        # -- update config
        # vel = 0.2
        # omega = 0.3
        # #deltaT = time.time() - oldTime
        # deltaT = 0.001 # assumed 
        # oldTime = time.time()
        # #print (deltaT)
        # robot.velControlKin(vel, omega, deltaT)

        # -- Previous implementation: set the configuration directly
        # if (configIdx == configLen):
        #     configIdx = configIdx - 1
        # robot.setConfig(ttConfigs[configIdx])
        """
        q = robot.getConfig()
        print 'Robot Config   : ', q
        if configIdx == configLen:
            print "Reached last Config!"
            if (rTime == None) :
                rTime = time.time()
            else:
                nowTime = time.time()
                if (rTime - nowTime > 10):
                    break
        else:
            print 'Required Config: ', ttConfigs[configIdx]
        q2f = [ '{0:.2f}'.format(elem) for elem in q]
        strng = "Robot configuration: " + str(q2f)
        vis.addText("textConfig", strng)

        ## Checking collision
        collisionFlag = False
        #for i,j in collisionChecker.collisionTests():
        #    if i[1].collides(j[1]):
        #        collisionFlag = True
        #        strng = "Object "+i[0].getName()+" collides with "+j[0].getName()
        #        print(strng)
        #        vis.addText("textCol", strng)
        #        vis.setColor("textCol", 0.8500, 0.3250, 0.0980)
        collRT0 = collisionChecker.robotTerrainCollisions(world.robot(0), world.terrain(0))
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
