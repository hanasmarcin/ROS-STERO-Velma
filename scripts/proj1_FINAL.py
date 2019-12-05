#!/usr/bin/env python

import roslib; roslib.load_manifest('stero_velma')
import rospy
import math
import numpy
import tf
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError


TOOL_DIST = 0.272

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
    Tf_table2.p[1] += 0.2
    Tf_table2.p[2] = 1.19
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
        'right_arm_2_joint':1, 'right_arm_3_joint':1, 'right_arm_4_joint':1.1, 'right_arm_5_joint':-1.6,
        'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
    q_map_goal_2 = {'torso_0_joint':-1.5, 'right_arm_0_joint':1, 'right_arm_1_joint':-1.1,
        'right_arm_2_joint':1, 'right_arm_3_joint':1.2, 'right_arm_4_joint':1.1, 'right_arm_5_joint':-1.6,
        'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
    q_map_goal_v2 = {'torso_0_joint':0, 'right_arm_0_joint':0.6, 'right_arm_1_joint':-1.3,
        'right_arm_2_joint':1, 'right_arm_3_joint':1.2, 'right_arm_4_joint':1.1, 'right_arm_5_joint':-1.4,
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
 

    dest_q = [0.6*math.pi,0.6*math.pi,0.6*math.pi,0]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), dest_q):
        exitError(9)

    print "move right:", dest_q
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(9)
 
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

#    rospy.sleep(10.0)

    dest_q = [0, 0, 0, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
        exitError(9)

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
    tool_org_tf = velma.getTf("B", "Tr")
    tool_tf = velma.getTf("B", "Tr")
    tool_tf.p[2] = Tf_object1.p[2]
    tf_diff = PyKDL.diff(tool_tf, Tf_object1, 1.0)
    alpha = math.atan2(tf_diff.vel[1], tf_diff.vel[0])
    print tf_diff.vel
    tf_diff.vel[0] -= TOOL_DIST*math.cos(alpha)
    tf_diff.vel[1] -= TOOL_DIST*math.sin(alpha)
    tf_dest = PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, alpha), tool_tf.p+tf_diff.vel+PyKDL.Vector(0, 0, 0.1))
    print tf_dest



    #exitError(0)
    print "Moving right wrist to pose defined in world frame..."
    #T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
    if not velma.moveCartImpRight([tf_dest], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(tf_dest, velma.getTf("B", "Tr"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.2 or T_B_T_diff.rot.Norm() > 0.2:
        exitError(15)

    dest_q = [0.6*math.pi,0.6*math.pi,0.6*math.pi,0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)

    print "Moving right wrist to pose defined in world frame..."
    #T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
    if not velma.moveCartImpRight([tool_org_tf], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(tool_org_tf, velma.getTf("B", "Tr"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.2 or T_B_T_diff.rot.Norm() > 0.2:
        exitError(15)
    rospy.sleep(0.5)

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)

    print "Moving to valid position, using planned trajectory."
    goal_constraint_1 = qMapToConstraints(q_map_goal_2, 0.01, group=velma.getJointGroup("impedance_joints"))
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
    if not isConfigurationClose(q_map_goal_2, js[1]):
        exitError(6)

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

    tool_org_tf = velma.getTf("B", "Tr")
    tool_tf = velma.getTf("B", "Tr")
    tool_tf.p[2] = Tf_table2.p[2]
    tf_diff = PyKDL.diff(tool_tf, Tf_table2, 1.0)
    alpha = math.atan2(tf_diff.vel[1], tf_diff.vel[0])
    print tf_diff.vel
    tf_diff.vel[0] -= TOOL_DIST*math.cos(alpha)
    tf_diff.vel[1] -= TOOL_DIST*math.sin(alpha)
    tf_dest = PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, alpha), tool_tf.p+tf_diff.vel+PyKDL.Vector(0, 0, 0.15))
    print tf_dest

    #exitError(0)
    print "Moving right wrist to pose defined in world frame..."
    #T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
    if not velma.moveCartImpRight([tf_dest], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(tf_dest, velma.getTf("B", "Tr"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.2 or T_B_T_diff.rot.Norm() > 0.2:
        exitError(15)

    dest_q = [0, 0, 0, 0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)

    print "Moving right wrist to pose defined in world frame..."
    #T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.3 , 1.3 ))
    if not velma.moveCartImpRight([tool_org_tf], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)
    print "Calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(tool_org_tf, velma.getTf("B", "Tr"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.2 or T_B_T_diff.rot.Norm() > 0.2:
        exitError(15)
    rospy.sleep(0.5)
    #if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
    #    exitError(9)

#end plan
