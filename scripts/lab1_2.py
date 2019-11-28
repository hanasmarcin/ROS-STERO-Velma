#!/usr/bin/env python

import roslib; roslib.load_manifest('stero_velma')
import rospy
import math
import numpy
import tf
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

def service(data):
    print data

R = 0.05
r = 0.3
p = 0.4
y = 0
x = 0.5
y = -0.4
z = 1

if __name__ == "__main__":
    rospy.init_node('lab1_2')
    
    rospy.sleep(0.5)

    print "This test/tutorial executes simple motions"\
         " of head."
 
    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"
 
    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
 
    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)
 
    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    q_map_start = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

    q_map_left = {'torso_0_joint':1.56, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

    q_map_right = {'torso_0_joint':-1.56, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)
 
    #print "Checking if the starting configuration is as expected..."
    #rospy.sleep(0.5)
    #js = velma.getLastJointState()
    #if not isConfigurationClose(q_map_left, js[1], tolerance=0.3):
    #    print "This test requires starting pose:"
    #    print q_map_starting
    #    exitError(10)
 
    rospy.sleep(1.0)

    print "Moving to the start..."
    velma.moveJoint(q_map_start, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_start, js[1], tolerance=0.1):
        exitError(10)
 
    print "moving head to position: up"
    q_dest = (0, -0.4)
    velma.moveHead(q_dest, 1.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    print "Moving to the left..."
    velma.moveJoint(q_map_left, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_left, js[1], tolerance=0.1):
        exitError(10)
 
    print "moving head to position: left up"
    q_dest = (1.56, -0.4)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)
 
    print "moving head to position: left down"
    q_dest = (1.56, 1)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)
 
    print "moving head to position: down"
    q_dest = (0, 1)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(9)

    print "Moving to the right..."
    velma.moveJoint(q_map_right, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_right, js[1], tolerance=0.1):
        exitError(10)

    print "moving head to position: right down"
    q_dest = (-1.56, 1)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(9)
 
    print "moving head to position: right up"
    q_dest = (-1.56, -0.4)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(9)
 
    print "moving head to position: up"
    q_dest = (0,-0.4)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(14)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(15)

    print "Moving to the middle..."
    velma.moveJoint(q_map_start, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_start, js[1], tolerance=0.1):
        exitError(10)

    print "moving head to position: middle"
    q_dest = (0, 0.1)
    velma.moveHead(q_dest, 1.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    print "Moving to the left..."
    velma.moveJoint(q_map_left, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_left, js[1], tolerance=0.1):
        exitError(10)
 
    print "moving head to position: left middle"
    q_dest = (1.56, 0.1)
    velma.moveHead(q_dest, 6.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: middle"
    q_dest = (0, 0.1)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    print "Moving to the right..."
    velma.moveJoint(q_map_right, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_right, js[1], tolerance=0.1):
        exitError(10)
 
    print "moving head to position: right middle"
    q_dest = (-1.56, 0.1)
    velma.moveHead(q_dest, 6.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(9)
 
    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 6.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(14)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(15)

    print "Moving to the start..."
    velma.moveJoint(q_map_start, 9.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(6)
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_start, js[1], tolerance=0.1):
        exitError(10)

 
    exitError(0)