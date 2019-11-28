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

    velma = VelmaInterface()
    velma.waitForInit()
    Tf_object1 = velma.getTf("B", "object1")
    Tf_table1 = velma.getTf("B", "table1")
    Tf_table2 = velma.getTf("B", "table2")

    p = Planner(velma.maxJointTrajLen())

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

#start plan
     # define some configurations
    q_map_goal = {'torso_0_joint':0, 'right_arm_0_joint':1, 'right_arm_1_joint':-1.3,
        'right_arm_2_joint':1, 'right_arm_3_joint':1, 'right_arm_4_joint':1.1, 'right_arm_5_joint':-1.7,
        'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
    q_map_start = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
 
 
    rospy.sleep(0.5)
 
    print "This test/tutorial executes complex motions"\
        " in Joint Impedance mode. Planning is used"\
        " in this example.\n"
 
    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
 
    print "Waiting for Planner initialization..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit(timeout_s=10.0):
        print "Could not initialize Planner"
        exitError(2)
    ocl = OctomapListener("/octomap_binary")
    rospy.sleep(0.5)
    octomap = ocl.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    print "Planner initialization ok!"
 
    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(3)
 
    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
 

    #dest_q = [0.6*math.pi,0.6*math.pi,0.6*math.pi,0]
    #velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    #if velma.waitForHandLeft() != 0:
    #    exitError(8)
    #rospy.sleep(0.5)
    #if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
    #    exitError(9)

    #print "move right:", dest_q
    #velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    #if velma.waitForHandRight() != 0:
    #    exitError(8)
    #rospy.sleep(0.5)
    #if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
    #    exitError(9)
 
    print "Planning motion to the goal position using set of all joints..."
 
    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(q_map_goal, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint_1], "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_goal, js[1]):
        exitError(6)
 
    rospy.sleep(10.0)

    #dest_q = [0, 0, 0, 0]
    #velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    #if velma.waitForHandRight() != 0:
    #    exitError(8)
    #rospy.sleep(0.5)
    #if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
    #    exitError(9)

    rospy.sleep(1)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    rospy.sleep(0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
 
    rospy.sleep(0.5)
 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(12)

    print "Moving right wrist to pose defined in world frame..."
    #T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
    T_B_Trd = Tf_object1
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
    print T_B_T_diff
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(15)

    dest_q = [0.6*math.pi,0.6*math.pi,0.6*math.pi,0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(9)

#end plan
