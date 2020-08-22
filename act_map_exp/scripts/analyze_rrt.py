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
from analyze_pose_errors import _loadPoseError


rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

init(autoreset=True)

pose_e_nm = 'pose_errors.txt'
rrt_stats_nm = 'rrt_stats.txt'
kMinOutIter = -1
kSecPerOutIter = 5.0


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

    if 'ordered_subdir_nms' in ana_cfg:
        subdir_nms = ana_cfg['ordered_subdir_nms']
    else:
        subdir_nms = sorted([v for v in os.listdir(cfg_dir)
                             if os.path.isdir(os.path.join(cfg_dir, v))])
    subdirs = [os.path.join(cfg_dir, v) for v in subdir_nms]
    print("Going to analyze variations {}".format(subdir_nms))

    has_exact_solution = []
    n_poses = []
    n_failed = []

    n_out_iters = []
    n_times_sec = []
    n_verts = []
    n_edges = []
    type_to_fail_rate = {}
    no_solution_types = []
    for idx, sd in enumerate(subdirs):
        print("- Process {}...".format(sd))
        rrt_stats_fn_i = os.path.join(sd, rrt_stats_nm)
        assert os.path.exists(rrt_stats_fn_i)
        rrt_stats_i = np.loadtxt(rrt_stats_fn_i)
        assert rrt_stats_i.shape[1] == 5
        nm_i = subdir_nms[idx]
        type_i = ana_cfg['types'][nm_i]

        n_out_iters.append(rrt_stats_i[:, 0])
        n_times_sec.append(
            [niter * kSecPerOutIter for niter in n_out_iters[-1].ravel().tolist()])
        n_verts.append(rrt_stats_i[:, 3])
        n_edges.append(rrt_stats_i[:, 4])

        if np.isinf(rrt_stats_i[-1, 2]):
            has_exact_solution.append(False)
            print(Fore.RED + '  no exact solution')
            assert type_i not in no_solution_types
            no_solution_types.append(type_i)
        else:
            has_exact_solution.append(True)

        pose_e_f_i = os.path.join(sd, pose_e_nm)
        if not os.path.exists(pose_e_f_i):
            print(Fore.RED + "  no pose found, plan failed? continue...")
            continue
        _, _, rot_e_i = _loadPoseError(pose_e_f_i)
        n_poses = len(rot_e_i)
        n_failed = sum([1 for v in rot_e_i if math.isnan(v)])
        fail_rate = 1.0 * n_failed / n_poses
        print("  {} out of {} poses failed: {}".format(
            n_failed, n_poses, fail_rate))
        assert type_i not in type_to_fail_rate
        type_to_fail_rate[type_i] = fail_rate

    # plotting
    fig = plt.figure(figsize=(12, 6))
    vert_ax = fig.add_subplot(121)
    if hide_x:
        vert_ax.get_xaxis().set_visible(False)
    vert_ax.set_ylabel('Num. Vertices')
    vert_ax.set_xlabel('Time (sec)')
    edge_ax = fig.add_subplot(122)
    edge_ax.set_ylabel('Num. Edges')
    edge_ax.set_xlabel('Time (sec)')
    if hide_x:
        edge_ax.get_xaxis().set_visible(False)
    for idx, nm in enumerate(subdir_nms):
        type_i = ana_cfg['types'][nm]
        vert_ax.semilogy(n_times_sec[idx], n_verts[idx], label=ana_cfg['labels'][type_i],
                         color=ana_cfg['colors'][type_i], linestyle=ana_cfg['linestyles'][type_i])
        vert_ax.yaxis.grid(linestyle='--')
        if kMinOutIter > 0:
            vert_ax.axvline(kMinOutIter, color='black', linestyle='dotted')
        edge_ax.semilogy(n_times_sec[idx], n_edges[idx], label=ana_cfg['labels'][type_i],
                         color=ana_cfg['colors'][type_i], linestyle=ana_cfg['linestyles'][type_i])
        edge_ax.yaxis.grid(linestyle='--')
        if kMinOutIter > 0:
            edge_ax.axvline(kMinOutIter, color='black', linestyle='dotted')
    print('saving error plots...')
    plt.legend(ncol=2, fontsize='small')
    fig.tight_layout()
    fig.savefig(os.path.join(cfg_dir, 'vert_edge_comp.png'),
                bbox_inches='tight', dpi=300)

    return type_to_fail_rate, no_solution_types


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('top_dir', type=str,
                        help='top folder that contains different variations')
    parser.add_argument('--base_ana_cfg', type=str, required=True,
                        help='base analysis configuration')
    parser.add_argument('--multiple', action='store_true', dest='multiple',
                        help='how to treat the top_dir')
    parser.set_defaults(multiple=False)
    args = parser.parse_args()

    base_ana_cfg = None
    assert os.path.exists(args.base_ana_cfg)
    shutil.copy2(args.base_ana_cfg, os.path.join(
        args.top_dir, 'base_analysis_cfg.yaml'))
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

        all_types = []
        all_type_to_fail_rate = []
        all_no_solution_types = []
        for idx, cfg_d_i in enumerate(cfg_dirs):
            cfg_nm_i = cfg_nms[idx]
            type_to_fail_rate_i, no_solution_types_i = analyzeSingleCfg(
                cfg_d_i, base_cfg=base_ana_cfg)
            all_types_i = sorted(list(type_to_fail_rate_i.keys()))
            if not all_types:
                all_types = all_types_i
            else:
                assert all_types == all_types_i

            all_type_to_fail_rate.append(type_to_fail_rate_i)
            all_no_solution_types.append(no_solution_types_i)

        with open(os.path.join(args.top_dir, 'overall_fail_rate.txt'), 'w') as f:
            f.write('{}\n'.format(' '.join(all_types)))
            for idx, cfg_nm_i in enumerate(cfg_nms):
                f.write(cfg_nm_i + " ")
                for type_i in all_types:
                    f.write("{:.2f}".format(
                        all_type_to_fail_rate[idx][type_i]))
                    if type_i in all_no_solution_types[idx]:
                        f.write("(X)")
                    f.write(" ")
                f.write('\n')
    else:
        analyzeSingleCfg(args.top_dir, base_ana_cfg)
