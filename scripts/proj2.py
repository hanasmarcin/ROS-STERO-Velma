#!/usr/bin/env python

import roslib; roslib.load_manifest('stero_velma')
import rospy
import math
import numpy
import tf
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError


q_map_goal = {'torso_0_joint':0, 'right_arm_0_joint':0.6, 'right_arm_1_joint':-1.8,
    'right_arm_2_joint':1.25, 'right_arm_3_joint':1.8, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.3,
    'right_arm_6_joint':-0.5, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

q_map_finish = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
    'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

def make_wrench(lx,ly,lz,rx,ry,rz):
    '''
    Function makes wrench.
    '''
    return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))


def initialize_velma():
    '''
    Function starts python interface and motors.
    '''
    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
 
    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(1)


def initialize_planner():
    '''
    Function starts planner and connects it with octomap.
    '''
    print "Waiting for Planner initialization..."
    global p
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit(timeout_s=10.0):
        print "Could not initialize Planner"
        exitError(2)
    ocl = OctomapListener("/octomap_binary")
    rospy.sleep(0.5)
    octomap = ocl.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    print "Planner initialization ok!"


def switch_to_jnt_imp():
    '''
    Function switches to joint impedance mode.
    '''
    print "Switch to jnt_imp mode..."
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(2)


def switch_to_cart_imp():
    '''
    Function switches to cartesian impedance mode.
    '''
    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(3)
    rospy.sleep(0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(4)
 
    rospy.sleep(0.5)
 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(5)


def move_right_fingers(angle):
    '''
    Function closes or opens left gripper by moving it's fingers.
    '''
    dest_q = [angle, angle, angle, math.pi]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(6)
    rospy.sleep(0.5)
    #if not isHandConfigurationClose(velma.getHandLeftCurrentConfiguration(), dest_q):
    #    exitError(7)


def move_left_fingers(close, check=True):
    '''
    Function closes or opens right gripper by moving it's fingers.
    '''
    if close:
    	dest_q = [0.6*math.pi, 0.6*math.pi, 0.6*math.pi, 0]
    else:
        dest_q = [0, 0, 0, 0]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(8)
    rospy.sleep(0.5)
    if check:
        if not isHandConfigurationClose(velma.getHandLeftCurrentConfiguration(), dest_q):
            exitError(9)
    else:
        if isHandConfigurationClose(velma.getHandLeftCurrentConfiguration(), dest_q):
            exitError(10)


def plan_and_move(q_map_goal):
    '''
    Function plans robot's movement to given goal (values for joints) and executes it.
    '''
    print "Planning motion to the goal."
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
            exitError(11)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue
 
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_goal, js[1]):
        exitError(12)


def set_lower_impedance():
    '''
    Function sets low impedance in X and Y axis.
    '''
    print "Set impedance to (40,40,1000,150,150,150) in tool frame."
    imp_list = [make_wrench(1000,1000,1000,150,150,150),
                make_wrench(500,500,1000,150,150,150),
                make_wrench(250,250,1000,150,150,150),
                make_wrench(40,40,1000,150,150,150)]
    if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5,2.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)


def set_higher_impedance():
    '''
    Function sets high impedance in X and Y axis.
    '''
    print "Set impedance to (1000,1000,1000,150,150,150) in tool frame."
    imp_list = [make_wrench(1000,1000,1000,150,150,150)]
    if not velma.moveCartImpRight(None, None, None, None, imp_list, [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)


def calculate_trajectory(Tf_cabinet):
    '''
    Function calculates points of opening door trajectory on an arc.
    '''
    traj = []
    wrist_traj = []
    R = 0.282 + 0.06
    Tf_correction = Tf_cabinet * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.4, -(R-0.05), 0.1))
    theta = Tf_cabinet.M.GetRPY()[2]

    for alpha in numpy.arange(-theta + 1.5*math.pi/4,-theta -0.5*math.pi/4 - 0.01, -0.5*math.pi/4):
        print alpha
        traj.append( PyKDL.Frame(Tf_cabinet.M, Tf_correction.p + PyKDL.Vector(R*math.cos(alpha), R*math.sin(alpha), 0)))
        wrist_traj.append( PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -alpha - math.pi/2 -0.45*math.pi/4), PyKDL.Vector(0, 0, 0)) )

    return traj, wrist_traj


def calculate_points(Tf_cabinet):
    '''
    Function calculates 3 points of opening door trajectory.
    '''
    traj = []
    wrist_traj = []
    tool_org_tf = velma.getTf("B", "Tr")
    traj.append(Tf_cabinet * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, math.pi), PyKDL.Vector(1, 0.05, 0)))
    traj.append(Tf_cabinet * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, math.pi), PyKDL.Vector(1, 0.7, 0)))
    traj.append(Tf_cabinet * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, math.pi), PyKDL.Vector(0.45, 1.2, 0))) #0.75 -2
    wrist_traj.append(PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -math.pi/9), PyKDL.Vector(0, 0, 0)))
    wrist_traj.append(PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -math.pi/3), PyKDL.Vector(0, 0, 0)))
    wrist_traj.append(PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, -math.pi/2), PyKDL.Vector(0, 0, 0)))
    return traj, wrist_traj


def recovery():
    '''
    Function used in case of PATH TOLERANCE error.
    '''
    print "Starting recovery.."
    switch_to_jnt_imp()
    switch_to_cart_imp()
    set_lower_impedance()
    print "po"
 
   
# K=40 N/m
# F_max = 7.2N
# tol = F_max/K = 0.18m
def move_right_wrist(tf_dest, time, tol=0.18):
    '''
    Function moves robot's right wrist to given transform.
    '''
    print "Moving right wrist to pose defined in world frame.."
    if not velma.moveCartImpRight(tf_dest, time, None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, 1), PyKDL.Vector(1, 1, 1))):
        exitError(13)
    error = velma.waitForEffectorRight()
    if error != 0 and error != -3: 
        exitError(14)
    if error == -3: #PATH_TOLERANCE_VIOLATED
        recovery()

    rospy.sleep(0.5)

def move_right_tool(tf_dest, tf_wrist, time, tol=0.20):
    '''
    Function moves robot's right wrist to given transform.
    '''
    print "Moving right wrist to pose defined in world frame.."
    if not velma.moveCartImpRight(tf_dest, time, tf_wrist, time, None, None, PyKDL.Wrench(PyKDL.Vector(5, 5, 5), PyKDL.Vector(5, 5, 5)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, 1), PyKDL.Vector(1, 1, 1))):
        exitError(13)
    error = velma.waitForEffectorRight()
    if error != 0 and error != -3: 
        exitError(14)
    if error == -3: #PATH_TOLERANCE_VIOLATED
        recovery()

    rospy.sleep(0.5)


def calculate_tf_dest(Tf_object, correction):
    '''
    Function calculates destination transform, used to pick up or put off an object.
    '''
    return PyKDL.Frame(Tf_object.M, Tf_object.p + correction)


def calculate_base_angle(Tf_object):
    '''
    Function calculates destination base angle.
    '''
    alpha = math.atan2(Tf_object.p[1], Tf_object.p[0])
    if (- 0.45*math.pi > alpha):
        alpha =  -0.45*math.pi
    elif (0.45*math.pi < alpha):
        alpha =  0.45*math.pi

    q_map_goal["torso_0_joint"] = alpha


if __name__ == "__main__":
    rospy.init_node('lab1_2')
    rospy.sleep(0.5)

    # Get object's and tables' transforms
    velma = VelmaInterface()
    velma.waitForInit()
    Tf_table = velma.getTf("B", "table")
    Tf_cabinet = velma.getTf("B", "cabinet")
    translation_tf = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, math.pi), PyKDL.Vector(0, 0.2, 0.12))
    Tf_cabinet_right = Tf_cabinet * translation_tf

    traj = calculate_trajectory(Tf_cabinet)

    # Initialize robot
    initialize_velma()
    initialize_planner()
    switch_to_jnt_imp()
 
    # Set robot to it's starting position
    move_right_fingers(0.58*math.pi)
    move_left_fingers(close=True)
    calculate_base_angle(Tf_table)
    plan_and_move(q_map_goal)
    rospy.sleep(1)

    switch_to_cart_imp()
    tool_org_tf = velma.getTf("B", "Tr")
    set_lower_impedance()
    move_right_wrist([PyKDL.Frame(Tf_cabinet_right.M, tool_org_tf.p + PyKDL.Vector(0, 0, 0.14))], [3])

    # Move robot's wrist to the door
    move_right_wrist([Tf_cabinet_right], [12], 0.25)

    # Move robot's wrist to the handle
    tool_org_tf = velma.getTf("B", "Tr")
    translation_tf = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0.1, 0.4, 0))
    move_tf = tool_org_tf * translation_tf
    move_right_wrist([move_tf], [12], 0.12)

    # Open the cabinet's door
    traj, wrist_traj = calculate_points(Tf_cabinet)
    
    print "MOVE 1"
    move_right_tool([traj[0]], [wrist_traj[0]], [9], 0.2)
    print "MOVE 2"
    move_right_tool([traj[1]], [wrist_traj[1]], [9], 0.2)
    print "MOVE 3"
    move_right_tool([traj[2]], [wrist_traj[2]], [9], 0.2)

    # Leave the handle
    move_right_fingers(0.66*math.pi)
    tool_org_tf = velma.getTf("B", "Tr")
    move_right_wrist([tool_org_tf * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(0, -0.2, 0))], [6], 0.20)
    tool_org_tf = velma.getTf("B", "Tr")
    move_right_wrist([tool_org_tf * PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(-0.1, 0, 0))], [6], 0.12)

    # Move to the end position
    switch_to_jnt_imp()
    plan_and_move(q_map_finish)
    exitError(0)
    