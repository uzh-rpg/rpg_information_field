#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import argparse
from datetime import datetime
from shutil import copyfile

import rospkg
import numpy as np
from colorama import init, Fore
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

import exp_utils as eu

import matplotlib
print(matplotlib.__version__)

_save_ext = ".pdf"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

test_prefix = ['gp_trace_', 'gp_info_det_', 'gp_info_meig_',
               'quad_info_det_', 'quad_trace_']
# position and the gradient
test_pos_fn = 'test_positions.txt'
test_pos_bearing_fn = 'test_positions_bearings_w.txt'
test_pos_grad_fn = 'pos_grad.txt'
# rotation and the gradient
test_rot_positions_fn = "rot_test_positions.txt"
test_rot_bearings_fn = "test_bearings_w.txt"
test_drot_bearings_fn = "drot_bearing_w"
ext = ".txt"


def plotMapPoints(ax, points, plt_rng):
    ax.scatter(points[0:-1:3, 0], points[0:-1:3, 1], points[0:-1:3, 2])
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)


def plotPosGradSingle(ax, vox_pos, grads, nm):
    norms = np.array([np.linalg.norm(r) for r in grads])
    max_n = np.amax(norms)
    grads = grads / max_n

    ax.quiver(
        vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
        grads[:, 0], grads[:, 1], grads[:, 2],
        length=0.2, normalize=True, arrow_length_ratio=0.4)

    # ax.set_title(nm)
    ax.view_init(azim=0, elev=90)
    ax.set_zticklabels([])


def plotRotationGradSingle(ax, pos, old_bearings, new_bearings):
    scale = 0.5
    s = scale * old_bearings + pos
    dir = (new_bearings - old_bearings) * scale
    nvalues = np.log10(np.array([np.linalg.norm(e) for e in dir]) + 1.0)
    nvalues = nvalues / np.amax(nvalues)
    colors = [[0, v, 0] for v in nvalues]
    arrow_colors = colors + [[0.2 * v[0], 0.2 * v[1], 0.2 * v[2]]
                             for v in colors for _ in (0, 1)]
    ax.quiver(
        s[:, 0], s[:, 1], s[:, 2],
        dir[:, 0], dir[:, 1], dir[:, 2],
        length=0.2, normalize=True, arrow_length_ratio=0.4, color=arrow_colors)
    ax.view_init(azim=0, elev=90)
    ax.set_zticklabels([])

if __name__ == '__main__':
    init(autoreset=True)
    rospack = rospkg.RosPack()

    parser = argparse.ArgumentParser()
    parser.add_argument('map', type=str,
                        help="<map>.txt under act_map/maps")
    parser.add_argument('--xrange', default=4.0, type=float,
                        help='Zero centered x range.')
    parser.add_argument('--yrange', default=4.0, type=float,
                        help='Zero centered y range.')
    parser.add_argument('--zrange', default=2.0, type=float,
                        help='Zero centered z range.')
    parser.add_argument('--pos_sample_step', type=float, default=0.5,
                        help="step to sample the positions")
    parser.add_argument('--n_rot_samples', type=int, default=10,
                        help="number of rotation samples")
    parser.add_argument('--n_pos_samples_for_rot', type=int, default=4,
                        help="number of rotation samples")
    parser.add_argument('--v', type=int, default=0,
                        help="verbosity level passed for glog")
    args = parser.parse_args()
    print(args)

    fn_base = os.path.basename(__file__).split('.')[0]

    abs_map_fn = eu.getMapAbsFn(args.map)
    abs_trace_dir, plots_dir = eu.createTraceDir(fn_base, args.map)
    abs_gp_vis_dir = eu.getGPVisAbsDir('fov45_fs30_lm1000_k10_fast')

    print(Fore.RED + ">>>>> Start expriment...")
    cmd = ['rosrun', 'act_map', 'exp_pose_gradient',
           '--xrange='+str(args.xrange/2),
           '--yrange='+str(args.yrange/2),
           '--zrange='+str(args.zrange/2),
           '--abs_map='+abs_map_fn,
           '--abs_trace_dir='+abs_trace_dir,
           '--abs_gp_vis_dir='+abs_gp_vis_dir,
           '--pos_sample_step='+str(args.pos_sample_step),
           '--n_rot_samples='+str(args.n_rot_samples),
           '--n_pos_samples_for_rot='+str(args.n_pos_samples_for_rot),
           '--v='+str(args.v),
           ]
    copyfile(abs_map_fn, abs_trace_dir+"/map_points.txt")
    subprocess.call(cmd)
    print(Fore.GREEN + "<<<<< Experiment done.")

    points = np.loadtxt(abs_trace_dir + "/map_points.txt")
    print(Fore.RED + ">>>>> Analysis and plotting...")

    print("- [TODO] Skip position gradient for now...")
    # print(Fore.GREEN + ">>>>> Position gradient...")
    # for nm_i in test_prefix:
        # print(Fore.RED + "Analyzing result {0}".format(nm_i))
        # test_positions = np.loadtxt(
            # os.path.join(abs_trace_dir, nm_i + test_pos_fn))
        # test_pos_bearings = np.loadtxt(os.path.join(abs_trace_dir,
                                                    # nm_i+test_pos_bearing_fn))
        # assert test_positions.shape == test_pos_bearings.shape
        # pos_gradients = np.loadtxt(
            # os.path.join(abs_trace_dir, nm_i + test_pos_grad_fn))
        # print("Loaded {0} gradients at {1} positions.".format(
            # pos_gradients.shape[0], test_positions.shape[0]))

        # fig = plt.figure(figsize=(8, 8))
        # ax = fig.add_subplot(111, projection='3d')
        # plotMapPoints(ax, points, args.xrange)
        # plotPosGradSingle(
            # ax, test_positions, pos_gradients, points,
            # args.xrange, r"{0}".format(nm_i))

    print("- Rotation gradient...")
    for nm_i in test_prefix:
        print(Fore.WHITE+"  Analyzing result {0}".format(nm_i))
        rot_test_positions = np.loadtxt(
            os.path.join(abs_trace_dir, nm_i+test_rot_positions_fn))
        rot_test_positions = rot_test_positions.reshape((-1, 3))
        n_test_pos = rot_test_positions.shape[0]
        rot_test_bearings = np.loadtxt(
            os.path.join(abs_trace_dir, nm_i+test_rot_bearings_fn))
        n_test_rot = rot_test_bearings.shape[0]
        print("  Find {0} positions where {1} rotation are tested".format(
            n_test_pos, n_test_rot))
        fig = plt.figure(figsize=(8, 8))
        fig.canvas.set_window_title(nm_i)
        ax = fig.add_subplot(111, projection='3d')
        plotMapPoints(ax, points, args.xrange)
        for pos_i in range(n_test_pos):
            print("  - pos {0}: {1}".format(pos_i, rot_test_positions[pos_i]))
            rot_new_bearings = np.loadtxt(
                os.path.join(abs_trace_dir,
                             nm_i+test_drot_bearings_fn+str(pos_i)+ext))
            assert rot_new_bearings.shape[0] == n_test_rot
            plotRotationGradSingle(ax, rot_test_positions[pos_i],
                                   rot_test_bearings, rot_new_bearings)

    print(Fore.GREEN + "<<<<< Analysis done.")

    plt.show()
