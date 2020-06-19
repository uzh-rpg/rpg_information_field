#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

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


def normalize(x):
    vmax = np.max(x)
    vmin = np.min(x)
    return np.array([(v - vmin)/(vmax-vmin) for v in x.tolist()])


def plotSingle(ax, vos_pos, points, plt_rng, res, nm):
    ax.scatter(points[0:-1:3, 0], points[0:-1:3, 1], points[0:-1:3, 2])
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    colors = [[0, v, 0] for v in res['nvalues']]
    arrow_colors = colors + [v for v in colors for _ in (0, 1)]
    # arrow_colors = colors + [[1, 0, 1] for v in colors for _ in (0, 1)]
    ax.quiver(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
              res['views'][:, 0], res['views'][:, 1], res['views'][:, 2],
              length=0.3, normalize=True, color=arrow_colors,
              arrow_length_ratio=0.7)
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)


def plotErrMap(ax, vox_pos, plt_rng, err, nm):
    ax.set_xlim(-plt_rng/2, plt_rng/2)
    ax.set_ylim(-plt_rng/2, plt_rng/2)
    ax.set_zlim(-plt_rng/2, plt_rng/2)
    n_err = normalize(np.array(err))
    colors = np.array([[v, 0, 0] for v in n_err])
    ax.scatter(vox_pos[:, 0], vox_pos[:, 1], vox_pos[:, 2],
               c=colors)
    ax.set_title(nm)
    ax.view_init(azim=0, elev=90)


init(autoreset=True)
_save_ext = ".pdf"
rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)
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
parser.add_argument('--animated', action='store_true',
                    help='whether to visualize it in animation')
parser.add_argument('--dt', default=0.01, type=float,
                    help='animation delta t')
args = parser.parse_args()
print(args)

map_dir = os.path.join(rospack.get_path('act_map'), 'maps')
abs_map_fn = os.path.join(map_dir, args.map+'_points_w.txt')
assert os.path.exists(abs_map_fn), "{0} does not exist".format(abs_map_fn)

dstr = datetime.now().strftime('%Y%m%d%H%M%S')
fn_base = os.path.basename(__file__).split('.')[0]
top_trace_dir = os.path.join(rospack.get_path("act_map"),
                             "trace/"+fn_base)
abs_trace_dir = os.path.join(top_trace_dir, args.map + '_' + dstr)
os.makedirs(abs_trace_dir)

plot_dirs = os.path.join(abs_trace_dir, 'plots')
os.makedirs(plot_dirs)

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
app_nm = 'app'
ext = '.txt'
view_pre = 'optim_view_'
val_pre = 'optim_value_'

vox_pos = np.loadtxt(abs_trace_dir + "/vox_pos" + ext)
points = np.loadtxt(abs_trace_dir + "/map_points" + ext)

closed_trace_res = {}
closed_trace_res['views'] = np.loadtxt(os.path.join(abs_trace_dir,
                                       view_pre+'trace_appr_zero_deriv'+ext))
closed_trace_res['values'] = np.loadtxt(os.path.join(abs_trace_dir,
                                        val_pre+'trace_appr_zero_deriv'+ext))

worst_trace_res = {}
worst_trace_res['views'] = np.loadtxt(
    os.path.join(abs_trace_dir, view_pre+'trace_appr_worst'+ext))
worst_trace_res['values'] = np.loadtxt(os.path.join(
    abs_trace_dir, val_pre+'trace_appr_worst'+ext))


gt_res = {}
app_res = {}
for mtype in test_mtypes:
    gt_nm_i = mtype + '_' + gt_nm
    gt_res_i = {}
    gt_res_i['views'] = np.loadtxt(os.path.join(abs_trace_dir,
                                                view_pre+gt_nm_i+ext))
    gt_res_i['values'] = np.loadtxt(os.path.join(abs_trace_dir,
                                                 val_pre+gt_nm_i+ext))
    gt_res[mtype] = gt_res_i

    app_nm_i = mtype + '_' + app_nm
    app_res_i = {}
    app_res_i['views'] = np.loadtxt(os.path.join(abs_trace_dir, 
                                                 view_pre+app_nm_i+ext))
    app_res_i['values'] = np.loadtxt(os.path.join(abs_trace_dir,
                                                  val_pre+app_nm_i+ext))
    app_res[mtype] = app_res_i


app_res['mineig']['nvalues'] =\
    normalize(np.log(app_res['mineig']['values']))
app_res['trace']['nvalues'] =\
    normalize(np.log10(np.log10(app_res['trace']['values'])))
# closed_trace_res['nvalues'] =\
    # normalize(np.log10(np.log10(closed_trace_res['values'])))
closed_trace_res['nvalues'] = app_res['trace']['nvalues']
worst_trace_res['nvalues'] =\
    normalize(np.log10(np.log10(worst_trace_res['values'])))
app_res['det']['nvalues'] =\
    normalize(np.log(app_res['det']['values']))

gt_res['mineig']['nvalues'] =\
    normalize(np.log(gt_res['mineig']['values']))
gt_res['trace']['nvalues'] =\
    normalize(np.log(gt_res['trace']['values']))
gt_res['det']['nvalues'] =\
    normalize(np.log(gt_res['det']['values']))

err_min_eig = eu.calErrorViews(app_res['mineig']['views'],
                               gt_res['mineig']['views'])
err_trace = eu.calErrorViews(app_res['trace']['views'],
                             gt_res['trace']['views'])
err_det = eu.calErrorViews(app_res['det']['views'],
                           gt_res['det']['views'])

err_diff_app = eu.calErrorViews(app_res['trace']['views'],
                                app_res['mineig']['views'])
err_diff_exact1 = eu.calErrorViews(gt_res['trace']['views'],
                                   gt_res['mineig']['views'])
err_diff_exact2 = eu.calErrorViews(gt_res['trace']['views'],
                                   gt_res['det']['views'])

### paper plots

# fig_save_trace_app = plt.figure(figsize=(6, 6))
# fig_save_mineig_app = plt.figure(figsize=(6, 6))
# fig_save_det_app = plt.figure(figsize=(6, 6))

# fig_save_trace_exa = plt.figure(figsize=(6, 6))
# fig_save_mineig_exa = plt.figure(figsize=(6, 6))
# fig_save_det_exa = plt.figure(figsize=(6, 6))

# ax = fig_save_mineig_app.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['mineig'],
           # r'$\sigma_{min}$ poly.')
# plt.tight_layout()
# fig_save_mineig_app.savefig(plot_dirs + '/mineigh_comp_app' + _save_ext, bbox_inches='tight')
# ax = fig_save_mineig_exa.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['mineig'],
           # r'$\sigma_{min}$')
# plt.tight_layout()
# fig_save_mineig_exa.savefig(plot_dirs + '/mineigh_comp_exa' + _save_ext, bbox_inches='tight')


# ax = fig_save_trace_app.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['trace'],
           # r'Trace poly.')
# plt.tight_layout()
# fig_save_trace_app.savefig(plot_dirs + '/trace_comp_app' + _save_ext, bbox_inches='tight')
# ax = fig_save_trace_exa.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
           # r'Trace')
# plt.tight_layout()
# fig_save_trace_exa.savefig(plot_dirs + '/trace_comp_exa' + _save_ext, bbox_inches='tight')


# ax = fig_save_det_exa.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['det'],
           # r'Det poly.')
# plt.tight_layout()
# fig_save_det_exa.savefig(plot_dirs + '/det_comp_exa' + _save_ext, bbox_inches='tight')
# ax = fig_save_det_app.add_subplot(111, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['det'],
           # r'Det')
# plt.tight_layout()
# fig_save_det_app.savefig(plot_dirs + '/det_comp_app' + _save_ext, bbox_inches='tight')


### visualiation for compare
axes = []
fig = plt.figure(figsize=(27, 14))
ax = fig.add_subplot(231, projection='3d')
plotSingle(ax, vox_pos, points, args.xrange, app_res['mineig'],
           r'$\sigma_{min}(I)$ Approx.')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(232, projection='3d')
plotSingle(ax, vox_pos, points, args.xrange, app_res['det'],
           r'$Det(I)$ Approx.')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(233, projection='3d')
plotSingle(ax, vox_pos, points, args.xrange, app_res['trace'],
           r'$Tr(I)$ Approx.')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(234, projection='3d')
plotSingle(ax, vox_pos, points, args.xrange, gt_res['mineig'],
           r'$\sigma_{min}(I)$ Exact')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(235, projection='3d')
plotSingle(ax, vox_pos, points, args.xrange, gt_res['det'],
           r'$Det(I)$ Exact')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)

ax = fig.add_subplot(236, projection='3d')
plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
           r'$Tr(I)$ Exact')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
axes.append(ax)


plt.tight_layout()
if args.animated:
    azim = np.arange(0, 360, 2)

    p_elev = np.arange(50, 75, 0.5)
    n_elev = np.flip(p_elev)
    elev = np.hstack((p_elev, n_elev))
    n_elev = elev.shape[0]

    fixed_elev = 60
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

# ax = fig.add_subplot(337, projection='3d')
# plotErrMap(ax, vox_pos, args.xrange, err_min_eig, 'mineig error')

# ax = fig.add_subplot(338, projection='3d')
# plotErrMap(ax, vox_pos, args.xrange, err_trace, 'trace error')

# ax = fig.add_subplot(339, projection='3d')
# plotErrMap(ax, vox_pos, args.xrange, err_det, 'det error')

# fig = plt.figure()

# ax = fig.add_subplot(231)
# plt.hist(err_min_eig, bins=eu.hist_bins)
# ax.set_title('mineig: appr vs exact')

# ax = fig.add_subplot(232)
# plt.hist(err_trace, bins=eu.hist_bins)
# ax.set_title('trace: appr vs exact')

# ax = fig.add_subplot(233)
# plt.hist(err_det, bins=eu.hist_bins)
# ax.set_title('det: appr vs exact')

# ax = fig.add_subplot(234)
# plt.hist(err_diff_app, bins=eu.hist_bins)
# ax.set_title('appr: trace vs mineig')

# ax = fig.add_subplot(235)
# plt.hist(err_diff_exact1, bins=eu.hist_bins)
# ax.set_title('exact: trace vs mineig')

# ax = fig.add_subplot(236)
# plt.hist(err_diff_exact2, bins=eu.hist_bins)
# ax.set_title('exact: trace vs det')

## specific compare on the trace
# fig = plt.figure()
# ax = fig.add_subplot(221, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, gt_res['trace'],
           # 'trace exact')
# ax = fig.add_subplot(222, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, app_res['trace'],
           # 'trace approx')
# ax = fig.add_subplot(223, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, closed_trace_res,
           # 'appr trace zero deriv')
# ax = fig.add_subplot(224, projection='3d')
# plotSingle(ax, vox_pos, points, args.xrange, worst_trace_res,
           # 'appr trace worst')
# plt.show()


print(Fore.GREEN + "<<<<< Analysis done.")

