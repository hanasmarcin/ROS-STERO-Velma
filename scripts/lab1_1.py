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
T_B_Trd = []

def calculate_tfs():
    '''
    Function calculates trajectory
    '''
    tf_list = []
    for alpha in numpy.arange(0, 2*math.pi, 2*math.pi/40):
	tf_list.append(PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(R*math.cos(alpha) ,R*math.sin(alpha) ,0)))

    base_tf = PyKDL.Frame(PyKDL.Rotation.RPY(r, p, y), PyKDL.Vector(x, y, z))
    for frame in tf_list:
	T_B_Trd.append(base_tf*frame)

    for frame in T_B_Trd:
	frame.M = PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 )


def initialize_robot():
    '''
    Function initializes robot in cart_imp mode
    '''
    global velma
    velma = VelmaInterface()
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)


if __name__ == "__main__":
    rospy.init_node('lab1_1')
    rospy.sleep(0.5)
    
    calculate_tfs()
    rospy.sleep(0.5)

    initialize_robot()
    rospy.sleep(0.5)
    
    print "Moving right wrist to pose defined in world frame..."
    print type(T_B_Trd[0])
    
    # Moving robot on given trajectory
    if not velma.moveCartImpRight(T_B_Trd, [0.3*t[0]+2 for t in enumerate(T_B_Trd)], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)

    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)
