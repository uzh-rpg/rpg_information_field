#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import numpy as np
import transformations as tfs


def skewv3(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def transformPt(T, pt):
    assert T.shape == (4, 4)
    return np.dot(T[0:3, 0:3], pt) + T[0:3, 3]


def wxyz2xyzw(quat):
    quat_new = np.zeros((4, ))
    quat_new[0:3], quat_new[3] = quat[1:4], quat[0]
    return quat_new


def xyzw2wxyz(quat):
    quat_new = np.zeros((4, ))
    quat_new[1:4], quat_new[0] = quat[0:3], quat[3]
    return quat_new


def exp_so3(v):
    """
    Grassia, F. S. (1998).
    Practical parameterization of rotations using the exponential map
    """
    angle = np.linalg.norm(v)

    if angle < np.power(np.finfo(float).eps, 0.25):
        na = 0.5 + angle * angle * 1.0 / 48.0
    else:
        na = np.sin(angle * 0.5) / angle

    ct = np.cos(angle * 0.5)

    T = tfs.quaternion_matrix([ct, na * v[0], na * v[1], na * v[2]])
    return T[0:3, 0:3]


def log_so3(R):
    assert R.shape == (3, 3)
    T = np.identity(4)
    T[0:3, 0:3] = R
    angle, axis, _ = tfs.rotation_from_matrix(T)

    return angle * axis


def exp_se3(v):
    vs = np.squeeze(v)
    assert vs.shape[0] == 6
    t = vs[0:3]
    rotv = vs[3:6]
    R = exp_so3(rotv)
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t

    return T


def log_se3(T):
    assert T.shape == (4, 4)
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    rotv = log_so3(R)

    return np.hstack((t, rotv))


def randomTranformation():
    T = tfs.random_rotation_matrix()
    T[0:3, 3] = np.random.rand(3)

    return T


def calTyaw(theta):
    return exp_se3(np.hstack((np.zeros(5), theta)))


def reorderAngle(angles):
    n_angle = angles.shape[0]
    # convert to 0-2pi
    angles_2pi = np.array([a % (2 * np.pi) for a in angles])
    for v in angles_2pi:
        assert v >= 0 and v <= 2 * np.pi

    # loop over and count wrap around
    bias = 0
    angles_re = angles_2pi
    for i in np.arange(1, n_angle):
        # calculate current bias
        s = angles_re[i-1] % (2 * np.pi)
        e = angles_re[i] % (2 * np.pi)
        if s > e and s > np.pi and (s - e) > np.pi:
            bias = bias + 2 * np.pi
        elif e > s and s < np.pi and (e - s) > np.pi:
            bias = bias - 2 * np.pi
        angles_re[i] += bias
    return angles_re



def leftJacobian(v):
    if np.linalg.norm(v) < 1e-10:
        return np.eye(3)
    s_v = skewv3(v)
    n_v = np.linalg.norm(v)
    I = np.identity(3)
    Jl = I + ((1 - np.cos(n_v)) / (n_v*n_v)) * s_v + \
        ((n_v - np.sin(n_v))/(n_v*n_v*n_v)) * np.dot(s_v, s_v)
    return Jl


def invLeftJacobian(v):
    if np.linalg.norm(v) < 1e-10:
        return np.eye(3)
    s_v = skewv3(v)
    n_v = np.linalg.norm(v)
    I = np.identity(3)
    Jl_inv = I - 0.5 * s_v + \
        (1/(n_v*n_v) - (1+np.cos(n_v))/(2*n_v*np.sin(n_v))) * np.dot(s_v, s_v)
    return Jl_inv


def rightJacobian(v):
    if np.linalg.norm(v) < 1e-10:
        return np.eye(3)
    s_v = skewv3(v)
    n_v = np.linalg.norm(v)
    I = np.identity(3)

    Jr = I - ((1 - np.cos(n_v)) / (n_v*n_v)) * s_v + \
        ((n_v - np.sin(n_v)) / (n_v * n_v * n_v)) * np.dot(s_v, s_v)

    return Jr


def invRightJacobian(v):
    if np.linalg.norm(v) < 1e-10:
        return np.eye(3)
    s_v = skewv3(v)
    n_v = np.linalg.norm(v)
    I = np.identity(3)

    Jr_inv = I + 0.5 * s_v + \
        (1/(n_v*n_v) - (1+np.cos(n_v))/(2*n_v*np.sin(n_v))) * np.dot(s_v, s_v)
    return Jr_inv



if __name__ == "__main__":
    axis = np.random.rand(3)
    axis = axis / np.linalg.norm(axis)
    angle = np.random.rand() - 0.5  # -0.5~0.5 no singularity
    rotv = angle * axis

    R = exp_so3(rotv)

    so3_vec = log_so3(R)

    print("Original so3 vector:\n{0}".format(rotv))
    print("Rotation matrix:\n{0}".format(R))
    print("so3 vector:\n{0}".format(so3_vec))

    t = np.random.rand(3)
    se3vec = np.hstack((t, rotv))
    T = exp_se3(se3vec)
    log_se3vec = log_se3(T)

    print("Original se3 vector:\n{0}".format(se3vec))
    print("Rotation matrix:\n{0}".format(T))
    print("so3 vector:\n{0}".format(log_se3vec))

    angles = np.array([0.1, -0.1, 0.5, 3.14, 5, 0.1])
    re_angles = reorderAngle(angles)
    print("raw angles are:\n {0}".format(angles))
    print("reordered angles are:\n {0}".format(re_angles))

    angles = np.array([0, np.pi-0.1, 0])
    re_angles = reorderAngle(angles)
    print("raw angles are:\n {0}".format(angles))
    print("reordered angles are:\n {0}".format(re_angles))

    angles = np.array([0, np.pi+0.1, 0])
    re_angles = reorderAngle(angles)
    print("raw angles are:\n {0}".format(angles))
    print("reordered angles are:\n {0}".format(re_angles))

    angles = np.array([0, np.pi-0.1, -np.pi+0.1, 0])
    re_angles = reorderAngle(angles)
    print("raw angles are:\n {0}".format(angles))
    print("reordered angles are:\n {0}".format(re_angles))

