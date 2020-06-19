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

import exp_utils as eu

init(autoreset=True)
_save_ext = ".pdf"
rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)
rospack = rospkg.RosPack()

parser = argparse.ArgumentParser()

parser.add_argument('--n_trials', type=int, default=50,
                    help="number of trials")
parser.add_argument('--n_pts', type=int, default=500,
                    help="number of random points")
parser.add_argument('--n_positions', type=int, default=50,
                    help="number of random positiosn")

parser.add_argument('--rdn_pts_size', type=float, default=10,
                    help="where to generate the points")
parser.add_argument('--pts_rad_low', type=float, default=0.8,
                    help="low radius")
parser.add_argument('--pts_rad_high', type=float, default=1.2,
                    help="high radius")
parser.add_argument('--rdn_pos_size', type=float, default=5,
                    help="where to generate the positions")

parser.add_argument('--v', type=int, default=0,
                    help="verbosity level passed for glog")
args = parser.parse_args()

dstr = datetime.now().strftime('%Y%m%d-%H%M%S')
fn_base = os.path.basename(__file__).split('.')[0]
top_trace_dir = os.path.join(rospack.get_path("act_map"),
                             "trace/"+fn_base)
abs_trace_dir = os.path.join(top_trace_dir, dstr)
os.makedirs(abs_trace_dir)

print(Fore.RED + ">>>>> Start expriment...")
cmd = ['rosrun', 'act_map', 'exp_comp_optim_orient',
       '--abs_trace_dir='+abs_trace_dir,
       '--n_trials='+str(args.n_trials),
       '--n_pts='+str(args.n_pts),
       '--n_positions='+str(args.n_positions),
       '--rdn_pts_size='+str(args.rdn_pts_size),
       '--rdn_pos_size='+str(args.rdn_pos_size),
       '--pts_rad_low='+str(args.pts_rad_low),
       '--pts_rad_high='+str(args.pts_rad_high),
       '--v='+str(args.v),
       ]
subprocess.call(cmd)
print(Fore.GREEN + "<<<<< Experiment done.")

test_mtypes = ['trace', 'det', 'mineig']
gt_nm = 'exact'
app_nm = 'app'
ext = '.txt'

gt_views = {}
app_views = {}
for mtype in test_mtypes:
    gt_views[mtype] = np.loadtxt(
        os.path.join(abs_trace_dir, 'optim_view_'+mtype+'_'+gt_nm+ext))
    app_views[mtype] = np.loadtxt(
        os.path.join(abs_trace_dir, 'optim_view_'+mtype+'_'+app_nm+ext))

err_views = {}
for mtype in test_mtypes:
    err_views[mtype] = eu.calErrorViews(gt_views[mtype], app_views[mtype])

for mtype in test_mtypes:
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.hist(err_views[mtype], bins=eu.hist_bins)
    ax.set_title(mtype)

plt.show()
