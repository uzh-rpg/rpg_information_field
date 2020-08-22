#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import argparse
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

_save_ext = ".png"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)


def plotSingle(ax, vox_pos, points, plt_rng, res, nm=None):
    ax.scatter(points[0:-1:3, 0], points[0:-1:3, 1], points[0:-1:3, 2])
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    colors = [[0, v, 0] for v in res['nvalues']]
    # head_cscale: 0.2 for darker header to be visible
    # change it to 1.0 to have arrows of the same color
    head_cscale = 1.0
    arrow_colors = colors + [[head_cscale * v[0], head_cscale * v[1], head_cscale * v[2]]
                             for v in colors for _ in (0, 1)]
    ax.quiver(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
              res['views'][:, 0], res['views'][:, 1], res['views'][:, 2],
              length=0.3, normalize=True, color=arrow_colors,
              arrow_length_ratio=0.6)
    if nm:
        ax.set_title(nm)
    ax.view_init(azim=0, elev=90)
    ax.set_zticklabels([])


def plotErrMap(ax, vox_pos, plt_rng, err, nm):
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    n_err = eu.normalize(np.array(err))
    colors = np.array([[v, 0, 0] for v in n_err])
    ax.scatter(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
               c=colors)
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)


def loadViewAndValues(nm, abs_trace_dir, view_pre, val_pre, ext):
    data = {}
    data['views'] = np.loadtxt(os.path.join(abs_trace_dir,
                                            view_pre+nm+ext))
    data['values'] = np.loadtxt(os.path.join(abs_trace_dir,
                                             val_pre+nm+ext))
    return data


def preprocessing(results):
    results['mineig']['nvalues'] =\
        eu.normalize(np.log(results['mineig']['values']))
    results['trace']['nvalues'] =\
        eu.normalize(np.log10(np.log10(results['trace']['values'])))
    results['det']['nvalues'] =\
        eu.normalize(np.log(results['det']['values']))


init(autoreset=True)
rospack = rospkg.RosPack()

parser = argparse.ArgumentParser()

parser.add_argument('map', type=str,
                    help="<map>.txt under act_map/maps")
parser.add_argument('--v', type=int, default=0,
                    help="verbosity level passed for glog")
parser.add_argument('--xrange', default=4.0, type=float,
                    help='Zero centered x range.')
parser.add_argument('--yrange', default=4.0, type=float,
                    help='Zero centered y range.')
parser.add_argument('--zrange', default=2.0, type=float,
                    help='Zero centered z range.')
parser.add_argument('--vox_res', default=0.1, type=float,
                    help='voxel resolution')
parser.add_argument('--check_ratio', default=0.01, type=float,
                    help='the ratio of voxels that will be computed')
parser.add_argument('--paper_plot', action='store_true',
                    help='do the paper plot')
# animation related
parser.add_argument('--animated', action='store_true', dest='animated',
                    help='whether to visualize it in animation')
parser.add_argument('--simple_plot', action='store_true', dest='simple_plot',
                    help='simplified plot for the video')
parser.add_argument('--dt', default=0.01, type=float, help='animation delta t')
parser.set_defaults(animated=False, simple_plot=False)
args = parser.parse_args()
print(args)

fn_base = os.path.basename(__file__).split('.')[0]

abs_map_fn = eu.getMapAbsFn(args.map)
abs_trace_dir, plots_dir = eu.createTraceDir(fn_base, args.map)

print(Fore.RED + ">>>>> Start expriment...")
cmd = ['rosrun', 'act_map', 'exp_optim_orient',
       '--abs_map='+abs_map_fn,
       '--abs_trace_dir='+abs_trace_dir,
       '--xrange='+str(args.xrange),
       '--yrange='+str(args.yrange),
       '--zrange='+str(args.zrange),
       '--vox_res='+str(args.vox_res),
       '--check_ratio='+str(args.check_ratio),
       '--v='+str(args.v),
       ]
copyfile(abs_map_fn, abs_trace_dir+"/map_points.txt")
subprocess.call(cmd)
print(Fore.GREEN + "<<<<< Experiment done.")

print(Fore.RED + ">>>>> Start analysis.")

test_mtypes = ['trace', 'det', 'mineig']
gt_nm = 'exact'
quad_app_nm = 'app'
gp_app_nm = 'gp_app'
ext = '.txt'
view_pre = 'optim_view_'
val_pre = 'optim_value_'

vox_pos = np.loadtxt(abs_trace_dir + "/vox_pos" + ext)
points = np.loadtxt(abs_trace_dir + "/map_points" + ext)

# closed_trace_res = loadViewAndValues('trace_appr_zero_deriv',
                                     # abs_trace_dir, view_pre, val_pre, ext)
# worst_trace_res = loadViewAndValues('trace_appr_worst',
                                    # abs_trace_dir, view_pre, val_pre, ext)

quad_app_res = {}
gp_app_res = {}
gt_res = {}
for mtype in test_mtypes:
    quad_nm_i = mtype + '_' + quad_app_nm
    quad_app_res[mtype] = loadViewAndValues(quad_nm_i, abs_trace_dir,
                                            view_pre, val_pre, ext)
    gp_nm_i = mtype + '_' + gp_app_nm
    gp_app_res[mtype] = loadViewAndValues(gp_nm_i, abs_trace_dir,
                                          view_pre, val_pre, ext)
    gt_nm_i = mtype + '_' + gt_nm
    gt_res[mtype] = loadViewAndValues(gt_nm_i, abs_trace_dir,
                                      view_pre, val_pre, ext)

preprocessing(quad_app_res)
preprocessing(gp_app_res)
preprocessing(gt_res)

nm_to_res = {'gt': gt_res, 'quad': quad_app_res, 'gp':gp_app_res}

# paper plots
if args.paper_plot:
    fig_save_trace_quad = plt.figure(figsize=(6, 6))
    fig_save_mineig_quad = plt.figure(figsize=(6, 6))
    fig_save_det_quad = plt.figure(figsize=(6, 6))

    fig_save_trace_gp = plt.figure(figsize=(6, 6))
    fig_save_mineig_gp = plt.figure(figsize=(6, 6))
    fig_save_det_gp = plt.figure(figsize=(6, 6))

    fig_save_trace_exa = plt.figure(figsize=(6, 6))
    fig_save_mineig_exa = plt.figure(figsize=(6, 6))
    fig_save_det_exa = plt.figure(figsize=(6, 6))

    # mineig
    ax = fig_save_mineig_quad.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, quad_app_res['mineig'],
               r'$\sigma_{min}$ poly.')
    fig_save_mineig_quad.tight_layout()
    fig_save_mineig_quad.savefig(
        plots_dir + '/mineig_comp_quad' + _save_ext, bbox_inches='tight')

    ax = fig_save_mineig_gp.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, gp_app_res['mineig'],
               r'$\sigma_{min}$')
    fig_save_mineig_gp.tight_layout()
    fig_save_mineig_gp.savefig(
        plots_dir + '/mineig_comp_gp' + _save_ext, bbox_inches='tight')

    ax = fig_save_mineig_exa.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, gt_res['mineig'],
               r'$\sigma_{min}$')
    fig_save_mineig_exa.tight_layout()
    fig_save_mineig_exa.savefig(
        plots_dir + '/mineig_comp_exa' + _save_ext, bbox_inches='tight')

    # trace 
    ax = fig_save_trace_quad.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, quad_app_res['trace'],
               r'Trace poly.')
    fig_save_trace_quad.tight_layout()
    fig_save_trace_quad.savefig(
        plots_dir + '/trace_comp_quad' + _save_ext, bbox_inches='tight')

    ax = fig_save_trace_gp.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, gp_app_res['trace'],
               r'Trace')
    fig_save_trace_gp.tight_layout()
    fig_save_trace_gp.savefig(
        plots_dir + '/trace_comp_gp' + _save_ext, bbox_inches='tight')

    ax = fig_save_trace_exa.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
               r'Trace')
    fig_save_trace_exa.tight_layout()
    fig_save_trace_exa.savefig(
        plots_dir + '/trace_comp_exa' + _save_ext, bbox_inches='tight')

    # det
    ax = fig_save_det_quad.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, quad_app_res['det'],
               r'Det')
    fig_save_det_quad.tight_layout()
    fig_save_det_quad.savefig(
        plots_dir + '/det_comp_quad' + _save_ext, bbox_inches='tight')

    ax = fig_save_det_gp.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, gp_app_res['det'],
               r'Det poly.')
    fig_save_det_gp.tight_layout()
    fig_save_det_gp.savefig(
        plots_dir + '/det_comp_gp' + _save_ext, bbox_inches='tight')

    ax = fig_save_det_exa.add_subplot(111, projection='3d')
    plotSingle(ax, vox_pos, points, args.xrange, gt_res['det'],
               r'Det poly.')
    fig_save_det_exa.tight_layout()
    fig_save_det_exa.savefig(
        plots_dir + '/det_comp_exa' + _save_ext, bbox_inches='tight')


    sys.exit(0)

axes = []
if args.simple_plot:
    fig = plt.figure(figsize=(12, 6))
    row_nm = ['det', 'trace']
    col_nm = ['gt', 'gp', 'quad']
    index = 1
    for r_nm_i in row_nm:
        for c_nm_i in col_nm:
            ax = fig.add_subplot(len(row_nm), len(col_nm), int(index), projection='3d')
            plotSingle(ax, vox_pos, points, args.xrange, nm_to_res[c_nm_i][r_nm_i],
                       nm='{}-{}'.format(c_nm_i, r_nm_i))
            ax.set_xticklabels([])
            ax.set_yticklabels([])
            ax.set_zticklabels([])
            axes.append(ax)
            index += 1
else:
    fig = plt.figure(figsize=(27, 21))

    row_nm = ['det', 'trace', 'mineig']
    col_nm = ['gt', 'gp', 'quad']
    index = 1
    for r_nm_i in row_nm:
        for c_nm_i in col_nm:
            ax = fig.add_subplot(len(row_nm), len(col_nm), int(index), projection='3d')
            plotSingle(ax, vox_pos, points, args.xrange, nm_to_res[c_nm_i][r_nm_i],
                       nm='{}-{}'.format(c_nm_i, r_nm_i))
            ax.set_xticklabels([])
            ax.set_yticklabels([])
            ax.set_zticklabels([])
            axes.append(ax)
            index += 1

    # ax = fig.add_subplot(331, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, quad_app_res['mineig'],
    #         r'$\sigma_{min}(I)$ Approx.')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(332, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, quad_app_res['det'],
    #         r'$Det(I)$ Approx.')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(333, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, quad_app_res['trace'],
    #         r'$Tr(I)$ Approx.')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(334, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, gp_app_res['mineig'],
    #         r'$\sigma_{min}(I)$ GP Approx.')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(335, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, gp_app_res['det'],
    #         r'$Det(I)$ GP Approx.')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(336, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, gp_app_res['trace'],
    #         r'$Tr(I)$ GP Approx.')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(337, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, gt_res['mineig'],
    #         r'$\sigma_{min}(I)$ Exact')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(338, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, gt_res['det'],
    #         r'$Det(I)$ Exact')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)

    # ax = fig.add_subplot(339, projection='3d')
    # plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
    #         r'$Tr(I)$ Exact')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    # axes.append(ax)


plt.tight_layout()
if args.animated:
    azim = np.arange(0, 360, 2)

    p_elev = np.arange(50, 75, 0.5)
    n_elev = np.flip(p_elev, axis=0)
    elev = np.hstack((p_elev, n_elev))
    n_elev = elev.shape[0]

    fixed_elev = 60
    for loop_i in range(1):
        for i in range(azim.shape[0]):
            a = azim[i]
            e = elev[i % n_elev]
            # print("Azim {0} and elev {1}".format(a, e))
            for ax in axes:
                ax.view_init(azim=a, elev=fixed_elev)
            plt.ion()
            plt.show()
            plt.pause(args.dt)
else:
    plt.show()

print(Fore.GREEN + "<<<<< Analysis done.")
