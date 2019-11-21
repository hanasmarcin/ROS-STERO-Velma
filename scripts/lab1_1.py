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
    rospy.init_node('lab1_1')
    rospy.sleep(0.5)

    tf_list = []
    for alpha in numpy.arange(0, 2*math.pi, 2*math.pi/40):
	tf_list.append(PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, 0), PyKDL.Vector(R*math.cos(alpha) ,R*math.sin(alpha) ,0)))

    T_B_Trd = []
    base_tf = PyKDL.Frame(PyKDL.Rotation.RPY(r, p, y), PyKDL.Vector(x, y, z))
    for frame in tf_list:
	T_B_Trd.append(base_tf*frame)

    for frame in T_B_Trd:
	frame.M = PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 )

    #br = tf.TransformBroadcaster()
    #rospy.sleep(1)
    #while not rospy.is_shutdown():
    #    for frame in T_B_Trd:
    #        br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()),
    #                 frame.M.GetQuaternion(),
    #                 rospy.Time.now(),
    #                 "kolo",
    #                 "world")
    #        rospy.sleep(0.2)
    #exitError(0)

    
    rospy.sleep(0.5)
    #exitError(0)

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
 
    rospy.sleep(0.5)
 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)
    


    print "Moving right wrist to pose defined in world frame..."
    print type(T_B_Trd[0])
    #T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector(T_B_Trd[0].p.x(), T_B_Trd[0].p.y(), T_B_Trd[0].p.z()))
    #print type(T_B_Trd)
    if not velma.moveCartImpRight(T_B_Trd, [0.3*t[0]+2 for t in enumerate(T_B_Trd)], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    #if not velma.moveCartImpRight(T_B_Trd, [2*t[0] for t in enumerate(T_B_Trd)], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #    exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)
    #print "calculating difference between desiread and reached pose..."
    #T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
    #print T_B_T_diff
    #if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
    #    exitError(10)

    print "hello world!"