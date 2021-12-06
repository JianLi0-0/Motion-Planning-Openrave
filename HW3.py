#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtosg')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('/home/parallels/planning/HW3/hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    # planner = RaveCreateModule(env,'RRTconnect')
    planner = RaveCreateModule(env,'BiRRT')
    env.AddModule(planner,args='')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot)
    # startconfig = [-0.15,0.075,0,-1.008,0,-0.11,0]
    # goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
  
    #set start config
    startconfig = [-0.15,0.075,0,-1.008,0,-0.11,0]
    goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
    command_str = (str(startconfig) + " " + str(goalconfig)).replace(',',' ').replace('[','').replace(']','')

    manip = robot.SetActiveManipulator('leftarm') #whatever arm you're using
    robot.SetDOFValues(startconfig, manip.GetArmIndices())
    robot.GetController().SetDesired(robot.GetDOFValues())

    waitrobot(robot)

    with env:
        ### YOUR CODE HERE ###
        print(command_str)
        start_time = time.time()
        planner.SendCommand('PlanningInterface '+ command_str)
        print("time:"+str(time.time()-start_time))
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

