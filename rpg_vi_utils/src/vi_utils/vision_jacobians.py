#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import numpy as np
import transformations as tfs
import tfs_utils as tu
from camera import Camera


def calK(fx, fy, cx, cy):
    K = np.zeros(3, 3)
    K[0, 0] = fx
    K[0, 2] = cx
    K[1, 1] = fy
    K[1, 2] = cy
    K[2, 2] = 1.0

    return K


def duDpc(fx, fy, p_c):
    x = p_c[0]
    y = p_c[1]
    z = p_c[2]
    z_inv = 1 / z
    z_inv2 = z_inv * z_inv

    jac = np.zeros((2, 3))
    jac[0, 0] = fx * z_inv
    jac[0, 2] = -fx * x * z_inv2
    jac[1, 0] = 0
    jac[1, 1] = fy * z_inv
    jac[1, 2] = -fy * y * z_inv2

    return jac


def dpt(T):
    """
    y = T * pt
    => dy/dpt
    """
    assert T.shape == (4, 4) or T.shape == (3, 3)
    return T[0:3, 0:3]


def inverse(T):
    invT = np.linalg.inv(T)
    return invT


def dptDse3_global(T, pt):
    """
    y = Exp(eps) * T * pt
    => dy / deps
    """

    jac = np.zeros((3, 6))
    jac[0:3, 0:3] = np.eye(3)
    jac[0:3, 3:6] = -1.0 * tu.skewv3(tu.transformPt(T, pt))

    return jac


def dptDse3_local(T, pt):
    """
    y = T * Exp(eps) * pt
    => dy/deps
    """
    jac = np.zeros((3, 6))
    jac[0:3, 0:3] = np.eye(3)
    jac[0:3, 3:6] = -1.0 * tu.skewv3(pt)
    jac = np.dot(T[0:3, 0:3], jac)

    return jac


def duDpw(fx, fy, p_w, T_w_c):
    """
    u = proj(Tcw * p_w)
    """
    T_c_w = tfs.inverse_matrix(T_w_c)
    p_c = tu.transformPt(T_c_w, p_w)

    return np.dot(duDpc(fx, fy, p_c), dpt(T_c_w))


def duDse3_global(fx, fy, p_w, T_w_c):
    """
    u = proj((Exp(eps) * T_w_c)^(-1)*p_w)
    u ~= proj(T_c_w * Exp(-eps)*p_w)
    """
    T_c_w = inverse(T_w_c)
    p_c = tu.transformPt(T_c_w, p_w)

    return np.dot(duDpc(fx, fy, p_c), -dptDse3_local(T_c_w, p_w))


def _dnorm_dv(v):
    '''
    norm = L2(v)
    '''

    norm_v = np.linalg.norm(v)
    assert norm_v >= 1e-4

    dim = v.shape[0]
    deriv = np.zeros((1, dim))

    for i in range(dim):
        deriv[0, i] = v[i] / norm_v

    return deriv


def _dinvnorm_dv(v, norm_v=None):
    '''
    invnorm = 1.0 / L2(v)
    '''

    if norm_v is None:
        norm_v = np.linalg.norm(v)

    assert norm_v >= 1e-4
    denom = np.power(norm_v, 3)

    dim = v.shape[0]
    deriv = np.zeros((1, dim))

    for i in range(dim):
        deriv[0, i] = -v[i] / denom

    return deriv


def df_dv(v):
    '''
    The jacobian of bearing vector with respect to the original vector
    f = normalize(v)
    '''

    norm = np.linalg.norm(v)
    dim = v.shape[0]
    invnorm = 1 / norm

    deriv = np.zeros((dim, dim))

    for row_i in range(dim):
        deriv[row_i, :] = _dinvnorm_dv(v, norm) * v[row_i]
        deriv[row_i, row_i] += invnorm

    return deriv


def dfDse3_global(p_w, T_w_c):
    T_c_w = inverse(T_w_c)
    p_c = tu.transformPt(T_c_w, p_w)

    return np.dot(df_dv(p_c), -dptDse3_local(T_c_w, p_w))


def dfDpw(p_w, T_w_c):
    T_c_w = inverse(T_w_c)
    p_c = tu.transformPt(T_c_w, p_w)

    return np.dot(df_dv(p_c), dpt(T_c_w))


if __name__ == '__main__':
    pt = [2, 3, 4]
    T = np.eye(4)
    T[0:3, 3] = [1, 2, 3]

    #%% basic test
    print("Pt:\n{0}, T:\n{1}".format(pt, T))

    pt1 = tu.transformPt(T, pt)
    print("Transformed point:\n{0}".format(pt1))

    dpt1_dse3 = dptDse3_global(T, pt)
    print("dpt1/dse:\n{0}".format(dpt1_dse3))

    dpt1_dpt = dpt(T)
    print("dpt1/dpt:\n{0}".format(dpt1_dpt))


    #%% test the jacobian of camera projection
    eps = 1e-4
    cam = Camera.createTestCam()
    pc = np.array([0.1, 0.5, 2.0])
    du_dpc_ana = duDpc(cam.fx, cam.fy, pc)
    u0 = cam.project3d(pc)
    assert u0 is not None
    du_dpc_num = np.zeros((2, 3))
    for i in range(3):
        pc[i] = pc[i] + eps
        up = cam.project3d(pc)
        pc[i] = pc[i] - 2 * eps
        ud = cam.project3d(pc)
        du_dpc_num[:, i] = (up - ud) / (2 * eps)
        pc[i] = pc[i] + eps

    print("Analytical jacobian is:\n{0}".format(du_dpc_ana))
    print("Numerical jacobian is:\n{0}".format(du_dpc_num))

    #%% test pose point transformation
    eps = 1e-8
    pt = np.random.rand(3)
    T = tu.randomTranformation()
    dpt_dse3_ana_local = dptDse3_local(T, pt)
    dpt_dse3_ana_global = dptDse3_global(T, pt)

    jac_local = np.zeros((3, 6))
    jac_global = np.zeros((3, 6))
    for i in range(6):
        eps_vec = np.zeros(6)
        eps_vec[i] = eps

        # local
        Tp = np.dot(T, tu.exp_se3(eps_vec))
        ptp = tu.transformPt(Tp, pt)
        Tm = np.dot(T, tu.exp_se3(-eps_vec))
        ptm = tu.transformPt(Tm, pt)
        jac_local[:, i] = (ptp - ptm) / (2 * eps)

        # global
        Tp = np.dot(tu.exp_se3(eps_vec), T)
        ptp = tu.transformPt(Tp, pt)
        Tm = np.dot(tu.exp_se3(-eps_vec), T)
        ptm = tu.transformPt(Tm, pt)
        jac_global[:, i] = (ptp - ptm) / (2 * eps)

    print("The jacobian for local perturbation:")
    print("Analytical:\n{0}\nNumerical:\n{1}".format(dpt_dse3_ana_local,
                                                     jac_local))
    print("The jacobian for global perturbation:")
    print("Analytical:\n{0}\nNumerical:\n{1}".format(dpt_dse3_ana_global,
                                                     jac_global))

    #%% test jacobian with respect to the point
    pt = np.random.rand(3)
    T = tu.randomTranformation()
    dpt_ana = dpt(T)

    jac = np.zeros((3, 3))
    for i in range(3):
        pt[i] = pt[i] + eps
        ptp = tu.transformPt(T, pt)
        pt[i] = pt[i] - 2 * eps
        ptm = tu.transformPt(T, pt)
        pt[i] = pt[i] + eps
        jac[:, i] = (ptp - ptm) / (2 * eps)

    print("Ana vs Num for point jacobian:\n{0}\n{1}".format(dpt_ana, jac))

    #%% test jacobian of image coordinates
    T_w_c = tu.randomTranformation()
    p_c = np.hstack((np.random.rand(2) - 0.5, np.random.rand() + 2))
    T_c_w = tfs.inverse_matrix(T_w_c)
    p_w = tu.transformPt(T_w_c, p_c)
    u = cam.project3d(p_c)
    assert u is not None

    du_dse3_global_ana = duDse3_global(cam.fx, cam.fy, p_w, T_w_c)
    jac_global = np.zeros((2, 6))
    for i in range(6):
        eps_vec = np.zeros(6)
        eps_vec[i] = eps
        # global
        Tp = np.dot(tu.exp_se3(eps_vec), T_w_c)
        ptp = tu.transformPt(tfs.inverse_matrix(Tp), p_w)
        up = cam.project3d(ptp)
        Tm = np.dot(tu.exp_se3(-eps_vec), T_w_c)
        ptm = tu.transformPt(tfs.inverse_matrix(Tm), p_w)
        um = cam.project3d(ptm)
        jac_global[:, i] = (up - um) / (2 * eps)

    print("The jacobian for global perturbation du/dse3:")
    print("Analytical:\n{0}\nNumerical:\n{1}".format(du_dse3_global_ana,
                                                     jac_global))

    du_dpw_ana = duDpw(cam.fx, cam.fy, p_w, T_w_c)
    jac = np.zeros((2, 3))
    for i in range(3):
        p_w[i] = p_w[i] + eps
        up = cam.project3d(tu.transformPt(T_c_w, p_w))
        p_w[i] = p_w[i] - 2 * eps
        um = cam.project3d(tu.transformPt(T_c_w, p_w))
        p_w[i] = p_w[i] + eps
        jac[:, i] = (up - um) / (2 * eps)

    print("Ana vs Num for du/dpw:\n{0}\n{1}".format(du_dpw_ana, jac))

    #%% test bearing vector jacobians
    dim = np.random.randint(1, 20)
    v = np.random.rand(dim)

    norm_v = np.linalg.norm(v)
    invnorm_v = 1.0 / norm_v
    f = v / norm_v

    jac_ana_norm = _dnorm_dv(v)
    jac_ana_invnorm = _dinvnorm_dv(v)
    jac_ana_f = df_dv(v)

    jac_num_norm = np.zeros((1, dim))
    jac_num_invnorm = np.zeros((1, dim))
    jac_num_f = np.zeros((dim, dim))
    for i in range(dim):
        v[i] = v[i] + eps
        norm_v_p = np.linalg.norm(v)
        invnorm_v_p = 1 / norm_v_p
        f_v_p = v / norm_v_p

        v[i] = v[i] - 2 * eps
        norm_v_m = np.linalg.norm(v)
        invnorm_v_m = 1 / norm_v_m
        f_v_m = v / norm_v_m

        jac_num_norm[0, i] = (norm_v_p - norm_v_m) / (2 * eps)
        jac_num_invnorm[0, i] = (invnorm_v_p - invnorm_v_m) / (2 * eps)
        jac_num_f[i] = (f_v_p - f_v_m) / (2 * eps)

        v[i] += eps

    print("Ana vs Num for dnorm/dv:\n{0}\n{1}".format(jac_ana_norm, jac_num_norm))
    print("Ana vs Num for dinvnorm/dv:\n{0}\n{1}".format(jac_ana_invnorm, jac_num_invnorm))
    print("Ana vs Num for df/dv:\n{0}\n{1}".format(jac_ana_f, jac_num_f))
    assert np.allclose(jac_ana_f, jac_num_f)
