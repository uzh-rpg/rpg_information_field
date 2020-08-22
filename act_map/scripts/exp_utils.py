#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
import rospkg
from datetime import datetime

hist_bins = np.arange(0, 180, 10).tolist()
rospack = rospkg.RosPack()


def createTraceDir(exp_type, exp_name):
    dstr = datetime.now().strftime('%Y%m%d%H%M%S')
    top_trace_dir = os.path.join(rospack.get_path("act_map"),
                                 "trace/"+exp_type)
    abs_trace_dir = os.path.join(top_trace_dir, exp_name + '_' + dstr)
    os.makedirs(abs_trace_dir)
    plots_dir = os.path.join(abs_trace_dir, 'plots')
    os.makedirs(plots_dir)

    return abs_trace_dir, plots_dir


def getMapAbsFn(map_name, pkg='act_map'):
    map_dir = os.path.join(rospack.get_path('act_map'), 'maps')
    abs_map_fn = os.path.join(map_dir, map_name+'_points_w.txt')
    assert os.path.exists(abs_map_fn), "{0} does not exist".format(abs_map_fn)

    return abs_map_fn


def getGPVisAbsDir(profile_name):
    vis_dir = os.path.join(rospack.get_path('act_map'),
                           'params/fov_approximator_gp')
    abs_gp_dir = os.path.join(vis_dir, profile_name)
    assert os.path.exists(abs_gp_dir), "{0} does not exist".format(abs_gp_dir)

    return abs_gp_dir


def normalize(x):
    vmax = np.max(x)
    vmin = np.min(x)
    return np.array([(v - vmin)/(vmax-vmin) for v in x.tolist()])


def calAngleBetween(v1, v2):
    if (not np.all(np.isfinite(v1))) or (not np.all(np.isfinite(v2))):
        print('Skipped invalid.')
        return None
    e_rad = np.arccos(v1.dot(v2) / (np.linalg.norm(v1)*np.linalg.norm(v2)))
    return np.rad2deg(e_rad)


def calErrorViews(views1, views2):
    err = []
    for v1, v2 in zip(views1, views2):
        err.append(calAngleBetween(v1, v2))
    err_fix = []
    for v in err:
        if np.isfinite(v):
            err_fix.append(v)
        else:
            err_fix.append(0.0)
    return err_fix
