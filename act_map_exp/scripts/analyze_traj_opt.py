#!/usr/bin/env python2

import os
import shutil
import argparse
from colorama import init, Fore, Style
import yaml
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

init(autoreset=True)

rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

summary_nm = 'ceres_summary.yaml'
total_cost_hist_nm = 'total_ceres_cost_history.txt'
info_cost_hist_nm = 'weighted_Info_cost_history.txt'


def _loadCostHistory(fn):
    data = np.loadtxt(fn)
    if data.size == 2:
        data = data.reshape((-1, 2))
    assert data.shape[1] == 2

    return data[:, 1].tolist()


def _analyzeSingleCfg(cfg_dir, base_cfg=None):
    print(Fore.RED + "==== process configuration {} ====".format(cfg_dir))
    analysis_cfg_fn = os.path.join(cfg_dir, 'analysis_cfg.yaml')
    assert os.path.exists(analysis_cfg_fn), analysis_cfg_fn
    with open(analysis_cfg_fn) as f:
        ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
    print(Style.DIM + "- Found analysis configuration {}".format(ana_cfg))
    if base_cfg:
        ana_cfg.update(base_cfg)
        print(Style.DIM + "- Effective analysis configuration {}".format(ana_cfg))

    subdir_nms = sorted([v for v in os.listdir(cfg_dir)
                         if os.path.isdir(os.path.join(cfg_dir, v))])
    subdirs = [os.path.join(cfg_dir, v) for v in subdir_nms]
    subdir_types = [ana_cfg['types'][v] for v in subdir_nms]
    print(Fore.GREEN + "Going to analyze variations:\n- {}".format('\n- '.join(subdir_nms)))

    #
    total_cost_hist = {}
    info_cost_hist = {}
    ceres_time = {}
    ceres_iter = {}
    for sd, sd_type in zip(subdirs, subdir_types):
        print(Style.DIM + '- {}'.format(sd))
        total_cost_fn = os.path.join(sd, total_cost_hist_nm)
        assert os.path.exists(total_cost_fn)
        info_cost_fn = os.path.join(sd, info_cost_hist_nm)
        assert os.path.join(info_cost_fn)

        total_cost_hist[sd_type] = _loadCostHistory(total_cost_fn)
        # total_cost_hist.append(_loadCostHistory(total_cost_fn))

        # TODO: hack due to bug in logging
        info_cost_hist_i = _loadCostHistory(info_cost_fn)
        info_cost_hist_i = info_cost_hist_i[-len(total_cost_hist[sd_type]):]

        info_cost_hist[sd_type] = info_cost_hist_i
        # info_cost_hist.append(info_cost_hist_i)

        assert len(total_cost_hist[sd_type]) == len(info_cost_hist[sd_type])

        summray_fn = os.path.join(sd, summary_nm)
        assert os.path.exists(summray_fn)
        with open(summray_fn, 'r') as f:
            ceres_summary = yaml.load(f, Loader=yaml.FullLoader)
            ceres_time[sd_type] = (
                ceres_summary['custom_solve_time'] - ceres_summary['custom_logger_time'])
            ceres_iter[sd_type] = ceres_summary['n_iter']
            # ceres_time.append(
            #     ceres_summary['custom_solve_time'] - ceres_summary['custom_logger_time'])

    return total_cost_hist, info_cost_hist, ceres_time, ceres_iter


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('top_dir', type=str,
                        help='top folder that contains different variations')
    parser.add_argument('--base_ana_cfg', type=str, required=True,
                        help='base analysis configuration')
    parser.add_argument('--multiple', action='store_true', dest='multiple',
                        help='how to treat the top_dir')
    parser.add_argument('--no_legend', action='store_false', dest='legend')
    parser.add_argument('--outdir', type=str, default=None)
    parser.add_argument('--plt_max_iter', type=int, default=-1)
    parser.set_defaults(multiple=False, legend=True)
    args = parser.parse_args()

    outdir = args.outdir if args.outdir else args.top_dir

    base_ana_cfg = None
    assert os.path.exists(args.base_ana_cfg)
    shutil.copy2(args.base_ana_cfg, os.path.join(args.top_dir, 'base_analysis_cfg.yaml'))
    with open(args.base_ana_cfg) as f:
        base_ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
    print("Base configuration for analysis is {}".format(base_ana_cfg))

    all_total_cost_history = []
    all_info_cost_history = []
    all_ceres_solve_time = []
    all_ceres_n_iter = []
    if args.multiple:
        cfg_nms = [v for v in sorted(os.listdir(args.top_dir))
                   if os.path.isdir(os.path.join(args.top_dir, v))]
        cfg_dirs = [os.path.join(args.top_dir, v) for v in cfg_nms]
        print(Style.BRIGHT + Fore.YELLOW + "1. Read results under {}:".format(args.top_dir))
        for v in cfg_nms:
            print(Fore.GREEN + "- {}".format(v))
        for cfg_d_i in cfg_dirs:
            total_cost_i, info_cost_i, ceres_time_i, ceres_iter_i = _analyzeSingleCfg(
                cfg_d_i, base_ana_cfg)
            all_total_cost_history.append(total_cost_i)
            all_info_cost_history.append(info_cost_i)
            all_ceres_solve_time.append(ceres_time_i)
            all_ceres_n_iter.append(ceres_iter_i)

        print(Style.BRIGHT + Fore.YELLOW + "2. Generting overall tables and plots")
        time_table_fn = os.path.join(outdir, 'overall_solve_times.txt')
        iter_table_fn = os.path.join(outdir, 'overall_solve_iterations.txt')
        time_iter_table_fn = os.path.join(outdir, 'overall_solve_time_iter.txt')
        print(Fore.GREEN + '- Solve times --> {}'.format(time_table_fn))
        print(Fore.GREEN + '- Solve iterations --> {}'.format(iter_table_fn))
        table_entries = base_ana_cfg['solve_time_table']
        print("Entries: {}".format(table_entries))

        average_ceres_time = {}
        average_ceres_iters = {}
        for type_i in table_entries:
            sum_ceres_time_i = 0.0
            sum_ceres_iter_i = 0
            for cfg_ceres_time, cfg_ceres_iters in zip(all_ceres_solve_time, all_ceres_n_iter):
                sum_ceres_time_i += cfg_ceres_time[type_i]
                sum_ceres_iter_i += int(cfg_ceres_iters[type_i])
            average_ceres_time[type_i] = sum_ceres_time_i * 1.0 / len(all_ceres_solve_time)
            average_ceres_iters[type_i] = sum_ceres_iter_i * 1.0 / len(all_ceres_solve_time)

        with open(time_table_fn, 'w') as f:
            f.write('{}\n'.format(' '.join(table_entries)))
            for cfg_nm, cfg_ceres_time in zip(cfg_nms, all_ceres_solve_time):
                f.write("{} ".format(cfg_nm))
                for type_i in table_entries:
                    f.write('{:.3f} '.format(cfg_ceres_time[type_i]))
                f.write('\n')
            f.write('aver. ')
            for type_i in table_entries:
                f.write('{:.3f} '.format(average_ceres_time[type_i]))
            f.write('\n')

        with open(iter_table_fn, 'w') as f:
            f.write('{}\n'.format(' '.join(table_entries)))
            for cfg_nm, cfg_ceres_iters in zip(cfg_nms, all_ceres_n_iter):
                f.write("{} ".format(cfg_nm))
                for type_i in table_entries:
                    f.write('{} '.format(int(cfg_ceres_iters[type_i])))
                f.write('\n')
            f.write('aver. ')
            for type_i in table_entries:
                f.write('{} '.format(average_ceres_iters[type_i]))
            f.write('\n')

        with open(time_iter_table_fn, 'w') as f:
            f.write('{}\n'.format(' '.join(table_entries)))
            for cfg_nm, cfg_ceres_time, cfg_ceres_iters in zip(cfg_nms,
                                                               all_ceres_solve_time, all_ceres_n_iter):
                f.write("{} ".format(cfg_nm))
                for type_i in table_entries:
                    f.write('{:.3f} ({})   '.format(cfg_ceres_time[type_i],
                                                    int(cfg_ceres_iters[type_i])))
                f.write('\n')
            f.write('aver. ')
            for type_i in table_entries:
                f.write('{:.3f} ({}) '.format(average_ceres_time[type_i],
                                              average_ceres_iters[type_i]))
            f.write('\n')

        print(Fore.GREEN + '- Plot cost vs. iterations')
        for plot_nm in sorted(base_ana_cfg['iter_cost_plot'].keys()):
            entries = base_ana_cfg['iter_cost_plot'][plot_nm]
            for cfg_nm, cfg_total_costs in zip(cfg_nms, all_info_cost_history):
                fig_fn = os.path.join(outdir, '{}_{}_iter_vs_cost.png'.format(plot_nm, cfg_nm))
                print('- {}: plotting {} --> {}'.format(cfg_nm, entries, fig_fn))
                fig = plt.figure(figsize=(8, 8))
                ax = fig.add_subplot(111)
                for type_i in entries:
                    total_cost_i = cfg_total_costs[type_i][:]
                    n_iter_i = len(total_cost_i)
                    if args.plt_max_iter > 0 and n_iter_i > args.plt_max_iter:
                        total_cost_i = total_cost_i[0:args.plt_max_iter]

                    color_i = base_ana_cfg['colors'][type_i]
                    label_i = base_ana_cfg['labels'][type_i]
                    line_i = base_ana_cfg['linestyles'][type_i]
                    denom_i = total_cost_i[0]
                    normalized_cost_i = [v / denom_i for v in total_cost_i]
                    iters_i = list(range(len(total_cost_i)))
                    ax.scatter(iters_i, normalized_cost_i, c=color_i, s=4)
                    ax.plot(iters_i, normalized_cost_i, linestyle=line_i,
                            c=color_i, label=label_i)

                ax.set_xlabel("Iter.")
                ax.set_ylabel("Normalized Perception Cost")

                plt.legend()
                fig.tight_layout()
                fig.savefig(fig_fn, bbox_inches='tight', dpi=300)
    else:
        _analyzeSingleCfg(args.top_dir, base_ana_cfg)
