#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('barrett_hand_controller')

import sys
import rospy
import math

import std_msgs.msg

import tf
from tf import *
from tf.transformations import * 
from tf2_msgs.msg import *

import PyKDL
import tf_conversions.posemath as pm
import copy
from scipy import optimize
import random

right_makrer_id=2

def locateMarker(T_T2_7, T_C_M):
        if len(T_T2_7) != len(T_C_M):
            return None
        if len(T_T2_7) < 2:
            return None

        T_70_7i = []
        T_M0_Mi = []

        for i in range(1, len(T_T2_7)):
            T_70_7i.append(T_T2_7[0].Inverse() * T_T2_7[i])
            T_M0_Mi.append(T_C_M[0].Inverse() * T_C_M[i])

        def estOrientation():
            def calc_R(rx, ry, rz):
                R_7_M = PyKDL.Frame(PyKDL.Rotation.EulerZYX(rx, ry, rz))
                diff = []
                for i in range(0,len(T_70_7i)):
                    diff.append(PyKDL.diff( T_70_7i[i] * R_7_M, R_7_M * T_M0_Mi[i] ))
                ret = [d.rot.x() for d in diff] + [d.rot.y() for d in diff] + [d.rot.z() for d in diff]
                return ret
            def f_2(c):
                """ calculate the algebraic distance between each contact point and jar surface pt """
                Di = calc_R(*c)
                return Di
            angle_estimate = random.random(), random.random(), random.random()
            angle_2, ier = optimize.leastsq(f_2, angle_estimate, maxfev = 10000)
            score = calc_R(angle_2[0],angle_2[1],angle_2[2])
            score_v = 0.0
            for s in score:
                score_v += s*s
            return [score_v, PyKDL.Frame(PyKDL.Rotation.EulerZYX(angle_2[0],angle_2[1],angle_2[2]))]

        best_score = 1000000.0
        best_R_7_M = PyKDL.Frame()
        for i in range(0, 10):
            score, R_7_M = estOrientation()
            if score < best_score:
                best_score = score
                best_R_7_M = copy.deepcopy(R_7_M)

        def estPos(R_7_M_est):
            rot_mx = copy.deepcopy(R_7_M_est.M)
            def calc_R(px, py, pz):
                R_7_M = PyKDL.Frame(rot_mx, PyKDL.Vector(px, py, pz))
                diff = []
                for i in range(0,len(T_70_7i)):
                    diff.append(PyKDL.diff( T_70_7i[i] * R_7_M, R_7_M * T_M0_Mi[i] ))
                ret = [d.vel.x() for d in diff] + [d.vel.y() for d in diff] + [d.vel.z() for d in diff]
                return ret
            def f_2(c):
                """ calculate the algebraic distance between each contact point and jar surface pt """
                Di = calc_R(*c)
                return Di
            pos_estimate = 0.0, 0.0, 0.0
            pos_2, ier = optimize.leastsq(f_2, pos_estimate, maxfev = 10000)
            score = calc_R(pos_2[0],pos_2[1],pos_2[2])
            score_v = 0.0
            for s in score:
                score_v += s*s
            return [score_v, PyKDL.Frame(rot_mx, PyKDL.Vector(pos_2[0], pos_2[1], pos_2[2]))]

        best_score = 1000000.0
        best_T_7_M = PyKDL.Frame()
        for i in range(0, 10):
            score, T_7_M = estPos(best_R_7_M)
            if score < best_score:
                best_score = score
                best_T_7_M = copy.deepcopy(T_7_M)

        return [best_score, best_T_7_M]

def meanOrientation(T):
    R = []
    for t in T:
        R.append( copy.deepcopy( PyKDL.Frame(t.M) ) )

    def calc_R(rx, ry, rz):
        R_mean = PyKDL.Frame(PyKDL.Rotation.EulerZYX(rx, ry, rz))
        diff = []
        for r in R:
            diff.append(PyKDL.diff( R_mean, r ))
        ret = [d.rot.x() for d in diff] + [d.rot.y() for d in diff] + [d.rot.z() for d in diff]
        return ret
    def f_2(c):
        """ calculate the algebraic distance between each contact point and jar surface pt """
        Di = calc_R(*c)
        return Di
    angle_estimate = R[0].M.GetEulerZYX()
    angle_2, ier = optimize.leastsq(f_2, angle_estimate, maxfev = 10000)
    score = calc_R(angle_2[0],angle_2[1],angle_2[2])
    score_v = 0.0
    for s in score:
        score_v += s*s
    return [score_v, PyKDL.Frame(PyKDL.Rotation.EulerZYX(angle_2[0],angle_2[1],angle_2[2]))]

if __name__ == "__main__":
    a = []
    for arg in sys.argv:
        a.append(arg)

    rospy.init_node('head_position', anonymous=True)

    tf_listener = tf.TransformListener()
    rospy.sleep(2.0)

    T_C_M = []
    T_T2_7 = []
    T_C_M_stable = PyKDL.Frame()
    T_T2_7_stable = PyKDL.Frame()
    stable_t = 0
    while True:
        rospy.sleep(0.1)
        try:
            pose = tf_listener.lookupTransform('camera', 'ar_marker_'+str(right_makrer_id), rospy.Time(0))
            T_C_M_current = pm.fromTf(pose)
            pose = tf_listener.lookupTransform('torso_link2', 'right_arm_7_link', rospy.Time(0))
            T_T2_7_current = pm.fromTf(pose)
        except:
            continue

        d1 = PyKDL.diff(T_C_M_stable, T_C_M_current)
        d2 = PyKDL.diff(T_T2_7_stable, T_T2_7_current)
        score = d2.vel.Norm() + d2.rot.Norm()
#        print score
        if score > 0.002:
            stable_t = 0
            T_C_M_stable = copy.deepcopy(T_C_M_current)
            T_T2_7_stable = copy.deepcopy(T_T2_7_current)
        else:
            stable_t += 1
            add = True
            if stable_t > 10:
                for t in T_T2_7:
                    d = PyKDL.diff(T_T2_7_current, t)
                    if d.rot.Norm() < 0.1:
                        add = False
                        break
                if add:
                    print "added"
                    T_C_M.append(copy.deepcopy(T_C_M_current))
                    T_T2_7.append(copy.deepcopy(T_T2_7_current))

        if len(T_C_M) > 4 or rospy.is_shutdown():
            break

    score,T_7_M = locateMarker(T_T2_7, T_C_M)

    q = T_7_M.M.GetQuaternion()
    print "PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s), PyKDL.Vector(%s,%s,%s))"%(q[0], q[1], q[2], q[3], T_7_M.p.x(), T_7_M.p.y(), T_7_M.p.z())
    print "score: %s"%(score)
    print T_7_M

    T_T2_C = []
    for i in range(0, len(T_C_M)):
        T_T2_C.append( T_T2_7[i] * T_7_M * T_C_M[i].Inverse() )

    mean_p = PyKDL.Vector()
    for i in range(0, len(T_T2_C)):
        mean_p += T_T2_C[i].p

    mean_p = mean_p * (1.0/len(T_T2_C))
    score,mean_R = meanOrientation(T_T2_C)
    print "mean rotation score: %s"%(score)

    br = tf.TransformBroadcaster()
    rospy.sleep(2.0)

    T_T2_C_est = PyKDL.Frame(copy.deepcopy(mean_R.M), mean_p)
    q = T_T2_C_est.M.GetQuaternion()
    print [T_T2_C_est.p.x(), T_T2_C_est.p.y(), T_T2_C_est.p.z()]
    print [q[0], q[1], q[2], q[3]]
    while not rospy.is_shutdown():
        br.sendTransform([T_T2_C_est.p.x(), T_T2_C_est.p.y(), T_T2_C_est.p.z()], [q[0], q[1], q[2], q[3]], rospy.Time.now(), "camera", "torso_link2")
        rospy.sleep(0.1)


