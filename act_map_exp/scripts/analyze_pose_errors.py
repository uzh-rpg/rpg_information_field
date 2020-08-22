#!/usr/bin/env python2

import os
import shutil
import argparse
from colorama import init, Fore
import yaml
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

init(autoreset=True)

pose_e_nm = 'pose_errors.txt'
Twc_nm = 'stamped_Twc.txt'

rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

def _replaceNan(values, rep):
    return [rep if math.isnan(v) else v for v in values]


def _loadPoses(pose_fn):
    times = []
    Twc = []
    data = np.loadtxt(pose_fn)
    assert data.shape[1] == 17

    times = data[:, 0].tolist()
    for l in data:
        Twc.append(l[1:].reshape((4, 4)))
    return times, Twc


def _loadPoseError(err_fn, max_trans_e_m=float('nan'),
                   max_rot_e_deg=float('nan')):
    names = []
    trans_e_m = []
    rot_e_deg = []
    with open(err_fn) as f:
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith('#'):
                continue
            elems = line.strip().split(' ')
            assert len(elems) == 3
            names.append(elems[0])

            te_i = float(elems[1])
            if not math.isnan(max_trans_e_m) and te_i > max_trans_e_m:
                trans_e_m.append(float('nan'))
            else:
                trans_e_m.append(te_i)

            re_i = float(elems[2])
            if not math.isnan(max_rot_e_deg) and re_i > max_rot_e_deg:
                rot_e_deg.append(float('nan'))
            else:
                rot_e_deg.append(re_i)

    return names, trans_e_m, rot_e_deg


def analyzeSingleCfg(cfg_dir, hide_x=False, base_cfg=None):
    print(Fore.RED + "==== process configuration {} ====".format(cfg_dir))
    analysis_cfg_fn = os.path.join(cfg_dir, 'analysis_cfg.yaml')
    assert os.path.exists(analysis_cfg_fn), analysis_cfg_fn
    with open(analysis_cfg_fn) as f:
        ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
    print("Found analysis configuration {}".format(ana_cfg))
    if base_cfg:
        ana_cfg.update(base_cfg)
        print("Effective analysis configuration {}".format(ana_cfg))

    subdir_nms = sorted([v for v in os.listdir(cfg_dir)
                         if os.path.isdir(os.path.join(cfg_dir, v))])
    subdirs = [os.path.join(cfg_dir, v) for v in subdir_nms]
    print("Going to analyze variations {}".format(subdir_nms))

    # load pose errors
    times = []
    pose_e_fns = []
    trans_e_m = []
    rot_e_deg = []
    for v in subdirs:
        print("- Process {}...".format(v))
        pose_e_f_i = os.path.join(v, pose_e_nm)
        pose_e_fns.append(pose_e_f_i)
        if not os.path.exists(pose_e_f_i):
            print(Fore.RED + "Pose error not found for {}, planning failed?".format(v))
            trans_e_i = []
            rot_e_i = []
            times_i = []
        else:
            _, trans_e_i, rot_e_i = _loadPoseError(
                pose_e_f_i, ana_cfg['max_trans_e_m'], ana_cfg['max_rot_e_deg'])
            times_i, _ = _loadPoses(os.path.join(v, Twc_nm))

        assert len(times_i) == len(trans_e_i), "{}: {} vs {}".format(
            v, len(times_i), len(trans_e_i))
        assert len(times_i) == len(rot_e_i), "{}: {} vs {}".format(
            v, len(times_i), len(rot_e_i))
        trans_e_m.append(trans_e_i)
        rot_e_deg.append(rot_e_i)
        times.append(times_i)

    fig = plt.figure(figsize=(16, 5))
    pos_ax = fig.add_subplot(121)
    if hide_x:
        pos_ax.get_xaxis().set_visible(False)
    pos_ax.set_ylabel('position error (m)')
    rot_ax = fig.add_subplot(122)
    rot_ax.set_ylabel('rotation error (deg)')
    if hide_x:
        rot_ax.get_xaxis().set_visible(False)
    for idx, nm in enumerate(subdir_nms):
        if not trans_e_m[idx]:
            print(Fore.RED + "Error for {} is empty, skip plotting.".format(nm))
            continue
        type_i = ana_cfg['types'][nm]
        pos_ax.plot(times[idx], trans_e_m[idx], label=ana_cfg['labels'][type_i],
                    color=ana_cfg['colors'][type_i], linestyle=ana_cfg['linestyles'][type_i])
        rot_ax.plot(times[idx], rot_e_deg[idx], label=ana_cfg['labels'][type_i],
                    color=ana_cfg['colors'][type_i], linestyle=ana_cfg['linestyles'][type_i])
    print('saving error plots...')
    plt.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(cfg_dir, 'errors_comp.png'), bbox_inches='tight', dpi=300)

    trans_e_raw = {}
    rot_e_raw = {}
    for idx, nm in enumerate(subdir_nms):
        if os.path.exists(pose_e_fns[idx]):
            _, trans_e_i_raw, rot_e_i_raw = _loadPoseError(pose_e_fns[idx])
        else:
            trans_e_i_raw = []
            rot_e_i_raw = []

        trans_e_raw[ana_cfg['types'][nm]] = trans_e_i_raw
        rot_e_raw[ana_cfg['types'][nm]] = rot_e_i_raw

    print(Fore.GREEN + '< Done.')

    return trans_e_raw, rot_e_raw


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('top_dir', type=str,
                        help='top folder that contains different variations')
    parser.add_argument('--base_ana_cfg', type=str, required=True,
                        help='base analysis configuration')
    parser.add_argument('--multiple', action='store_true', dest='multiple',
                        help='how to treat the top_dir')
    parser.add_argument('--no_legend', action='store_false', dest='legend')
    parser.add_argument('--plt_min_ratio', type=float, default=0.0)
    parser.add_argument('--plt_max_ratio', type=float, default=1.0)
    parser.set_defaults(multiple=False, legend=True)
    args = parser.parse_args()

    base_ana_cfg = None
    assert os.path.exists(args.base_ana_cfg)
    shutil.copy2(args.base_ana_cfg, os.path.join(args.top_dir, 'base_analysis_cfg.yaml'))
    with open(args.base_ana_cfg) as f:
        base_ana_cfg = yaml.load(f)
    print("Base configuration for analysis is {}".format(base_ana_cfg))

    if args.multiple:
        cfg_nms = [v for v in sorted(os.listdir(args.top_dir))
                   if os.path.isdir(os.path.join(args.top_dir, v))]
        cfg_dirs = [os.path.join(args.top_dir, v) for v in cfg_nms]
        print(Fore.YELLOW + "1. Analyzing configurations under {}:".format(args.top_dir))
        for v in cfg_nms:
            print(Fore.YELLOW + "- {}".format(v))
        acc_trans_e = {}
        acc_rot_e = {}
        for cfg_d_i in cfg_dirs:
            trans_e_i, rot_e_i = analyzeSingleCfg(cfg_d_i, base_cfg=base_ana_cfg)
            for k, v in trans_e_i.items():
                if k not in acc_trans_e:
                    acc_trans_e[k] = v
                else:
                    acc_trans_e[k].extend(v)
            for k, v in rot_e_i.items():
                if k not in acc_rot_e:
                    acc_rot_e[k] = v
                else:
                    acc_rot_e[k].extend(v)
        print(Fore.YELLOW + "<<< Finished all configurations.")
        print(Fore.YELLOW + "2. Gathered translation and rotation errors:")
        print("- translation errors:")
        for k, v in acc_trans_e.items():
            print('  - {}: {}'.format(k, len(v)))
        print("- rotation errors:")
        for k, v in acc_rot_e.items():
            print('  - {}: {}'.format(k, len(v)))

        plot_max_trans_e = 1.2 * base_ana_cfg['hist_max_trans_e']
        plot_max_rot_e = 1.2 * base_ana_cfg['hist_max_rot_e']

        ordered_types = base_ana_cfg['ordered_types']
        trans_errors = [acc_trans_e[v] for v in ordered_types]
        trans_errors = [_replaceNan(v, plot_max_trans_e) for v in trans_errors]
        trans_labels = [base_ana_cfg['labels'][v] for v in ordered_types]
        trans_colors = [base_ana_cfg['colors'][v] for v in ordered_types]

        rot_errors = [acc_trans_e[v] for v in ordered_types]
        rot_errors = [_replaceNan(v, plot_max_rot_e) for v in rot_errors]
        rot_labels = [base_ana_cfg['labels'][v] for v in ordered_types]
        rot_colors = [base_ana_cfg['colors'][v] for v in ordered_types]

        fig = plt.figure(figsize=(12, 6))
        axes = fig.subplots(1, 2, sharey=True)
        pos_ax, rot_ax = axes[0], axes[1]
        for idx, v in enumerate(ordered_types):
            pos_ax.hist(trans_errors[idx], label=trans_labels[idx], bins=50,
                        color=trans_colors[idx], linestyle=base_ana_cfg['linestyles'][v],
                        density=True, histtype='step', cumulative=True,
                        range=(0, plot_max_trans_e))
            rot_ax.hist(rot_errors[idx], label=rot_labels[idx], bins=50,
                        color=rot_colors[idx], linestyle=base_ana_cfg['linestyles'][v],
                        density=True, histtype='step', cumulative=True,
                        range=(0, plot_max_rot_e))
        pos_ax.set_xlabel('Position error (m)')
        rot_ax.set_xlabel('Rotation error (deg)')

        pos_ax.set_xlim([0, base_ana_cfg['hist_max_trans_e']])
        pos_ax.set_ylim([args.plt_min_ratio, args.plt_max_ratio + 0.01])

        rot_ax.set_xlim([0, base_ana_cfg['hist_max_rot_e']])
        rot_ax.set_ylim([args.plt_min_ratio, args.plt_max_ratio + 0.01])

        ystep = (args.plt_max_ratio- args.plt_min_ratio) / 4.0
        y_ticks = np.arange(args.plt_min_ratio, args.plt_max_ratio + 0.001, ystep)
        y_ticklabels = ['{}\%'.format(int(v * 100)) for v in y_ticks]

        pos_ax.set_yticks(y_ticks)
        pos_ax.set_yticklabels(y_ticklabels)

        if args.legend:
            plt.legend(loc='lower right', ncol=2, fontsize='small')

        fig.tight_layout()
        fig.savefig(os.path.join(args.top_dir, 'overall_hist.png'),
                    bbox_inches='tight', dpi=300)

    else:
        analyzeSingleCfg(args.top_dir, base_ana_cfg)
