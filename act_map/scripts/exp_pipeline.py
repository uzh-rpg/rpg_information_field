#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import os
import argparse
import subprocess
from shutil import copyfile, rmtree

import rospkg
from colorama import init, Fore
import matplotlib.pyplot as plt
from matplotlib import rc
from mpl_toolkits.mplot3d import Axes3D

init(autoreset=True)
_save_ext = ".pdf"
rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)
rospack = rospkg.RosPack()


parser = argparse.ArgumentParser()
parser.add_argument('traj', type=str,
                    help="<traj>.csv under act_map/trajectories")
parser.add_argument('map', type=str,
                    help="<map>.txt under act_map/maps")
parser.add_argument('--v', type=int, default=0,
                    help="verbosity level passed for glog")
parser.add_argument('--min_obs', type=int, default=5,
                    help="Number of observations to good observations")
args = parser.parse_args()

map_dir = os.path.join(rospack.get_path('act_map'), 'maps')
abs_map_fn = os.path.join(map_dir, args.map+'_points_w.txt')
assert os.path.exists(abs_map_fn), "{0} does not exist".format(abs_map_fn)

traj_dir = os.path.join(rospack.get_path('act_map'), 'trajectories')
abs_traj_fn = os.path.join(traj_dir, args.traj+'.csv')
assert os.path.exists(abs_traj_fn), "{0} does not exist".format(abs_traj_fn)

fn_base = os.path.basename(__file__).split('.')[0]
top_trace_dir = os.path.join(rospack.get_path("act_map"),
                             "trace/"+fn_base)
abs_trace_dir =\
    os.path.join(top_trace_dir,
                 args.map + '_' + os.path.basename(args.traj).split('.')[0])
if os.path.exists(abs_trace_dir):
    print(Fore.RED + "<<<<< Removing previous trace dir...")
    rmtree(abs_trace_dir)
os.makedirs(abs_trace_dir)

print(Fore.RED + ">>>>> Start expriment...")
cmd = ['rosrun', 'act_map', 'exp_pipeline',
       '--abs_traj='+abs_traj_fn,
       '--abs_map='+abs_map_fn,
       '--abs_trace_dir='+abs_trace_dir,
       '--v='+str(args.v),
       '--min_obs='+str(args.min_obs),
       ]
copyfile(abs_map_fn, abs_trace_dir+"/map_points.txt")
copyfile(abs_traj_fn, abs_trace_dir+"/traj.csv")
subprocess.call(cmd)
print(Fore.GREEN + "<<<<< Experiment done.")
