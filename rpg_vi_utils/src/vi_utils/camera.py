#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import numpy as np
import tfs_utils as tu


class Camera(object):
    """
    simple camera projection model
    """
    def __init__(self, fx, fy, cx, cy, w, h):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.half_wfov = np.arctan2(self.w/2.0, self.fx)
        self.half_hfov = np.arctan2(self.h/2.0, self.fy)

    def project3d(self, pt, margin=0.01):
        x = pt[0]
        y = pt[1]
        z = pt[2]

        if pt[2] < margin:
            return None

        u = np.zeros(2)
        u[0] = x / z * self.fx + self.cx
        u[1] = y / z * self.fy + self.cy

        if u[0] < 0 or u[0] > self.w:
            return None
        if u[1] < 0 or u[1] > self.h:
            return None

        return u

    def checkVisibilityBatch(self, pcs, margin=0.01):
        assert pcs.shape[1] == 3
        res = self.project3dBatch(pcs, margin)

        return np.array([int(v is not None) for v in res])

    def checkVisibilityBatchWorld(self, T_w_c, pws, margin=0.01):
        T_c_w = np.linalg.inv(T_w_c)
        pcs = np.array([tu.transformPt(T_c_w, v) for v in pws])
        assert pcs.shape[1] == 3
        res = self.project3dBatch(pcs, margin)

        return np.array([int(v is not None) for v in res])

    def project3DWorld(self, T_w_c, pw):
        T_c_w = np.linalg.inv(T_w_c)
        return self.project3d(tu.transformPt(T_c_w, pw))

    def project3dBatch(self, pts, margin=0.01):
        res = []
        for pt in pts:
            res.append(self.project3d(pt))
        return res

    def project3DWorldBatch(self, T_w_c, pws):
        T_c_w = np.linalg.inv(T_w_c)
        pcs = np.array([tu.transformPt(T_c_w, v) for v in pws])
        return self.project3dBatch(pcs)

    @staticmethod
    def filterInvisible(us, pts):
        viz_us = np.array([v for v in us if v is not None])
        viz_pts = np.array([pts[i] for i, v in enumerate(us) if v is not None])
        return viz_us, viz_pts

    @staticmethod
    def createTestCam():
        cam = Camera(407.424437, 407.504883, 340.911041, 240.253188, 752, 480)
        return cam


if __name__ == "__main__":
    cam = Camera.createTestCam()

    print("The horizontal and vertical FoVs are {0} and {1}".format(cam.half_wfov, cam.half_hfov))

    N = 500
    pts = np.zeros((N, 3))
    pts[:, 0] = np.random.uniform(-5, 5, N)
    pts[:, 1] = np.random.uniform(-5, 5, N)
    pts[:, 2] = np.random.uniform(1.0, 2.0, N)

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import plot_utils as pu
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], alpha=0.1)
    ax.scatter(0, 0, 0, 'x')
    ax.auto_scale_xyz([-5, 5], [-5, 5], [-5, 5])
    pu.plotCoordinateFrame3d(np.identity(4), ax)

    pxs = cam.project3dBatch(pts)
    viz_pxs, viz_pts = cam.filterInvisible(pxs, pts)
    ax.scatter(viz_pts[:, 0], viz_pts[:, 1], viz_pts[:, 2], c='r')
    ax = fig.add_subplot(122)
    ax.scatter(viz_pxs[:, 0], viz_pxs[:, 1])
