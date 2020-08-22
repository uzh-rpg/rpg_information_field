#!/usr/bin/env python

import os
import argparse
import yaml

import numpy as np
from colorama import init, Fore, Style
from matplotlib import rc
import matplotlib.pyplot as plt

import plot_utils as pu

init(autoreset=True)

rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)

kMetrics = ['det', 'mineig', 'trace']
kMetricsLabels = ['$\det$', '$\lambda_{min}$', '${Tr}$']
kSecToUs = 1.0e6
kPallete = [
    'blue', 'green', 'red', 'gold', 'purple', 'gray', 'cyan',
    'midnightblue', 'lime', 'lightcoral', 'darkgoldenrod', 'violet', 'dimgray',  'darkorange',
    'black'
]


def normalize(data, min_val=0.0, max_val=1.0):
    data_valid = [v for v in data if v is not None]
    dmax = np.max(data_valid)
    dmin = np.min(data_valid)
    ddata = dmax - dmin
    ddes = max_val - min_val
    return [(v - dmin) / ddata * ddes + min_val if v is not None else v for v in data]


def logAndNormalize(data, min_val=0.0, max_val=1.0):
    data_log = [np.log(v) if v > 0 else None for v in data]
    return normalize(data_log)


def readResults(res_dir, nm):
    file_nms = sorted([v for v in os.listdir(res_dir) if v.endswith('.txt') and nm in v])
    print("- Found files for map {}:\n  - {}".format(nm, '\n  - '.join(file_nms)))

    print("- read general info")
    gen_nm = "general_info_{}.txt".format(nm)
    general_info = {}
    if gen_nm in file_nms:
        data = np.loadtxt(os.path.join(res_dir, gen_nm))
        assert data.shape == (4,)
        general_info['n_vox'] = data[0]
        general_info['t_construct'] = data[1]
        general_info['ker_mem_kb'] = data[2]
        general_info['pc_mem_kb'] = data[3]

    print("- read fim")
    fim_vox_c_nm = 'fim_vox_c_{}.txt'.format(nm)
    fim = {}
    if fim_vox_c_nm in file_nms:
        fim_vox_c_fn = os.path.join(res_dir, fim_vox_c_nm)
        fim_map_fn = os.path.join(res_dir, "fim_map_{}.txt".format(nm))
        fim_pc_fn = os.path.join(res_dir, "fim_pc_{}.txt".format(nm))
        fim_vox_centers = np.loadtxt(fim_vox_c_fn)
        assert fim_vox_centers.shape[1] == 3
        n_fim = fim_vox_centers.shape[0]
        print(Style.DIM + "Found {} FIM".format(n_fim))
        fim_map = np.loadtxt(fim_map_fn)
        assert fim_map.shape[1] == 36
        assert n_fim == fim_map.shape[0]
        fim_pc = np.loadtxt(fim_pc_fn)
        assert fim_pc.shape[1] == 36
        assert n_fim == fim_pc.shape[0]

        fim['vox_c'] = fim_vox_centers
        fim['map'] = [v.reshape((6, 6)) for v in fim_map]
        fim['pc'] = [v.reshape((6, 6)) for v in fim_pc]

        fim_time_map_fn = os.path.join(res_dir, "fim_time_map_{}.txt".format(nm))
        fim_time_map = np.loadtxt(fim_time_map_fn)
        fim_time_pc_fn = os.path.join(res_dir, "fim_time_pc_{}.txt".format(nm))
        fim_time_pc = np.loadtxt(fim_time_pc_fn)
        fim['map_time'] = fim_time_map
        fim['map_time_mean'] = np.mean(fim_time_map)
        fim['pc_time'] = fim_time_pc
        fim['pc_time_mean'] = np.mean(fim_time_pc)
        print(Style.DIM + "Aver. Map: {}. Aver. PC: {}".format(
            fim['map_time_mean'], fim['pc_time_mean']))
    else:
        print(Fore.RED + "Nothing found.")

    print("- read query time")
    t_query = {}
    for m in kMetrics:
        t_map_fn = os.path.join(res_dir, 't_query_map_{}_{}.txt'.format(m, nm))
        if not os.path.exists(t_map_fn):
            print(Fore.RED + "* metric {} does not exist for query time".format(m))
            continue
        t_pc_fn = os.path.join(res_dir, 't_query_pc_{}_{}.txt'.format(m, nm))
        assert os.path.exists(t_pc_fn)
        t_map = np.loadtxt(t_map_fn)
        t_map_mean = np.mean(t_map)
        t_pc = np.loadtxt(t_pc_fn)
        t_pc_mean = np.mean(t_pc)
        assert t_map.size == t_pc.size
        print(Style.DIM + "* metric {}: {} samples, map aver. {}, pc aver. {}".format(
            m, t_map.size, t_map_mean, t_pc_mean))
        t_query[m] = {}
        t_query[m]['map'] = t_map.ravel().tolist()
        t_query[m]['map_mean'] = t_map_mean
        t_query[m]['pc'] = t_pc.ravel().tolist()
        t_query[m]['pc_mean'] = t_pc_mean

    print("- read optimal orientations")
    optim_orient = {}
    oo_vox_c_fn = os.path.join(res_dir, 'optim_orient_vox_c_{}.txt'.format(nm))
    assert os.path.exists(oo_vox_c_fn), oo_vox_c_fn
    oo_vox_c = np.loadtxt(oo_vox_c_fn)
    assert oo_vox_c.shape[1] == 3
    n_oo = oo_vox_c.shape[0]
    optim_orient['vox_c'] = oo_vox_c
    print(Style.DIM + "Total {} samples".format(n_oo))
    for m in kMetrics:
        oo_map_fn = os.path.join(res_dir, 'optim_orient_map_{}_{}.txt'.format(m, nm))
        if not os.path.exists(oo_map_fn):
            print(Fore.RED + "* metric {} does not exist for optimal orientations".format(m))
            continue
        else:
            print(Style.DIM + "* metric {}".format(m))
        oo_pc_fn = os.path.join(res_dir, 'optim_orient_pc_{}_{}.txt'.format(m, nm))
        assert os.path.exists(oo_pc_fn)

        optim_orient[m] = {}
        oo_map = np.loadtxt(oo_map_fn)
        assert oo_map.shape == (n_oo, 3)
        oo_pc = np.loadtxt(oo_pc_fn)
        assert oo_pc.shape == (n_oo, 3)
        optim_orient[m]['map'] = oo_map
        optim_orient[m]['pc'] = oo_pc

    print("- read metrics for continous motion")
    cont_metrics = {}
    cont_rot_fn = os.path.join(res_dir, 'metric_cont_rot_{}.txt'.format(nm))
    if os.path.exists(cont_rot_fn):
        cont_trans_fn = os.path.join(res_dir, 'metric_cont_trans_{}.txt'.format(nm))
        assert os.path.exists(cont_trans_fn)

        cont_metrics['rot'] = {}
        cont_rot = np.loadtxt(cont_rot_fn)
        assert cont_rot.shape[0] == 2
        print(Style.DIM + "{} rotations.".format(cont_rot.shape[1]))
        cont_metrics['rot']['map'] = cont_rot[0].ravel().tolist()
        cont_metrics['rot']['pc'] = cont_rot[1].ravel().tolist()

        cont_metrics['trans'] = {}
        cont_trans = np.loadtxt(cont_trans_fn)
        assert cont_trans.shape[0] == 2
        print(Style.DIM + "{} translations.".format(cont_trans.shape[1]))
        cont_metrics['trans']['map'] = cont_trans[0].ravel().tolist()
        cont_metrics['trans']['pc'] = cont_trans[1].ravel().tolist()
    else:
        print(Fore.RED + "Nothing found.")

    return {"general_info": general_info, 'fim': fim, 't_query': t_query,
            'optim_orient': optim_orient, 'cont_metrics': cont_metrics}


def _writeComplexityTable(nm_to_res, pc_res_key, selected_nms, nm_to_label, complexity_table_fn):
    sel_labels = [nm_to_label[v] for v in selected_nms]
    with open(complexity_table_fn, 'w') as f:
        f.write('# PC {}\n'.format(' '.join(sel_labels)))
        # construction time
        f.write('t_construct (sec) ')
        f.write('- ')
        for nm in selected_nms:
            f.write('{} '.format(nm_to_res[nm]['general_info']['t_construct']))
        f.write('\n')
        # memory
        f.write('memory (MB) ')
        f.write('{:.2f} '.format(nm_to_res[pc_res_key]['general_info']['pc_mem_kb'] / 1024.0))
        for nm in selected_nms:
            f.write('{:.2f} '.format(nm_to_res[nm]['general_info']['ker_mem_kb'] / 1024.0))
        f.write('\n')

        # query
        # fim
        f.write('# query (us)\n')
        f.write('fim ')
        f.write('{:.1f} '.format(nm_to_res[pc_res_key]['fim']['pc_time_mean'] * kSecToUs))
        for nm in selected_nms:
            fim_res = nm_to_res[nm]['fim']
            if 'map_time' not in fim_res:
                f.write('- ')
            else:
                f.write('{:.1f} '.format(fim_res['map_time_mean'] * kSecToUs))
        f.write('\n')
        # metrics
        for m in kMetrics:
            f.write('{} '.format(m))
            f.write('{:.1f} '.format(nm_to_res[pc_res_key]['t_query'][m]['pc_mean'] * kSecToUs))
            for nm in selected_nms:
                t_query = nm_to_res[nm]['t_query']
                if m not in t_query:
                    f.write('- ')
                else:
                    f.write('{:.1f} '.format(t_query[m]['map_mean'] * kSecToUs))
            f.write('\n')


def _computeAndWriteFIMDiff(nm_to_res, selected_nms, nm_to_label, top_save_dir=None):
    fim_fro_diff = {}
    sel_labels = [nm_to_label[v] for v in selected_nms]
    for nm in selected_nms:
        print('- calculating {}'.format(nm))
        fim_pc = nm_to_res[nm]['fim']['pc']
        fim_map = nm_to_res[nm]['fim']['map']
        fim_diff_perc = []
        for fim_pc_i, fim_map_i in zip(fim_pc, fim_map):
            fro_pc_i = np.linalg.norm(fim_pc_i)
            fro_dfim_i = np.linalg.norm(fim_map_i - fim_pc_i)
            fim_diff_perc.append(fro_dfim_i / fro_pc_i * 100)
        if top_save_dir:
            with open(os.path.join(top_save_dir, 'fim_fro_diff_perc_{}.txt'.format(nm)), 'w') as f:
                f.write('# each item is one percentage of FIM difference w.r.t. point cloud\n')
                f.write('{}'.format(' '.join(['{:.2f}'.format(v) for v in fim_diff_perc])))
        fim_fro_diff[nm] = fim_diff_perc
        print(Style.DIM + "* {}: {} ({})".format(nm, np.median(fim_diff_perc), np.std(fim_diff_perc)))

    if top_save_dir:
        print('- writing table')
        with open(os.path.join(top_save_dir, 'fim_fro_diff_table.txt'), 'w') as f:
            f.write('# Median (std): {}\n'.format(' '.join(sel_labels)))
            for nm in selected_nms:
                diff_perc = fim_fro_diff[nm]
                f.write("{} ({}) ".format(np.median(diff_perc), np.std(diff_perc)))

    return fim_fro_diff


def _boxplotFIMDiffs(nm_to_fim_diff_perc, names, nm_to_label, top_save_dir):
    xlabels = [nm_to_label[v] for v in names]
    data_labels = ['FIM Diff']
    colors = [kPallete[0]]

    data = []
    for nm in names:
        data.append(nm_to_fim_diff_perc[nm])

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111)
    pu.boxplot_compare(ax, xlabels, [data], data_labels, colors, legend=False)
    ax.set_ylabel("FIM diff. (\%)")
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
             rotation_mode="anchor")
    plt.tight_layout()
    fig.savefig(os.path.join(top_save_dir, 'fim_diffs_boxplot.png'), bbox_inches='tight')


def _computeAndWriteOptimalOrientDiff(nm_to_res, selected_nms, nm_to_label, top_save_dir=None):
    orient_diffs = {}
    for nm in selected_nms:
        print('- calculating {} ...'.format(nm))
        orient_diff_per_metric = {}
        oo_results = nm_to_res[nm]['optim_orient']
        for m in kMetrics:
            diffs_i = []
            for o_map, o_pc in zip(oo_results[m]['map'], oo_results[m]['pc']):
                cos_val = max(-1.0,
                              min(1.0,
                                  np.dot(o_map, o_pc) / (np.linalg.norm(o_map) * np.linalg.norm(o_pc))))
                diffs_i.append(np.rad2deg(np.arccos(cos_val)))
            print(Style.DIM + "{}: {} ({})".format(m, np.median(diffs_i), np.std(diffs_i)))
            orient_diff_per_metric[m] = diffs_i
        orient_diffs[nm] = orient_diff_per_metric

        if top_save_dir:
            with open(os.path.join(top_save_dir, 'orient_diffs_{}.txt'.format(nm)), 'w') as f:
                for m in kMetrics:
                    f.write('{} {}\n'.format(m, ' '.join([str(v)
                                                          for v in orient_diff_per_metric[m]])))

    return orient_diffs


def _boxplotOrientDiffs(orient_diffs, names, nm_to_label, top_save_dir):
    xlabels = kMetricsLabels
    data_labels = [nm_to_label[v] for v in names]
    colors = [kPallete[i] for i, v in enumerate(names)]

    data = []
    for nm in names:
        data_i = []
        for m in kMetrics:
            data_i.append(orient_diffs[nm][m])
        data.append(data_i)

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111)
    pu.boxplot_compare(ax, xlabels, data, data_labels, colors)
    ax.set_ylabel("Orientation diff. (deg)")
    plt.tight_layout()
    fig.savefig(os.path.join(top_save_dir, 'orient_diffs_boxplot.png'), bbox_inches='tight')


def _compareContinuousMotion(nm_to_res, selected_nms, nm_to_label, top_save_dir):
    pc_cont_motion_res = nm_to_res[selected_nms[0]]
    pc_rot_metrics = logAndNormalize(pc_cont_motion_res['cont_metrics']['rot']['pc'])
    pc_trans_metrics = logAndNormalize(pc_cont_motion_res['cont_metrics']['trans']['pc'])

    fig_rot = plt.figure(figsize=(8, 6))
    ax_rot = fig_rot.add_subplot(111)
    ax_rot.plot(pc_rot_metrics, label='Point Cloud')
    for nm_i in selected_nms:
        ax_rot.plot(logAndNormalize(nm_to_res[nm_i]['cont_metrics']
                                    ['rot']['map']), label=nm_to_label[nm_i])
    ax_rot.set_xticks([])
    ax_rot.set_xlabel('Continuous Rotation')
    ax_rot.set_ylabel('Normalized Det.')
    plt.legend()
    plt.tight_layout()
    fig_rot.savefig(os.path.join(top_save_dir, 'continuous_rotation.png'), bbox_inches='tight')

    fig_trans = plt.figure(figsize=(8, 6))
    ax_trans = fig_trans.add_subplot(111)
    ax_trans.plot(pc_trans_metrics, label='Point Cloud')
    for nm_i in selected_nms:
        ax_trans.plot(logAndNormalize(nm_to_res[nm_i]
                                      ['cont_metrics']['trans']['map']), label=nm_to_label[nm_i])
    ax_trans.set_xticks([])
    ax_trans.set_xlabel('Continuous Translation')
    # ax_trans.set_ylabel('Normalized Det.')
    # plt.legend()
    plt.tight_layout()
    fig_trans.savefig(os.path.join(top_save_dir, 'continuous_translation.png'), bbox_inches='tight')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--res_dir', required=True)
    parser.add_argument('--analyze_config', required=True)
    parser.add_argument('--save_dir', type=str, default='analysis_results')
    parser.add_argument('--pc_res_key', type=str, default='quad_info_0p5')
    parser.set_defaults(map_names=['quad_info', 'quad_trace', 'gp_info', 'gp_trace'])
    args = parser.parse_args()
    print(Fore.YELLOW + args.__dict__.__str__())

    with open(args.analyze_config, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
    print("Read configurations:\n{}".format(cfg))
    map_names = []
    map_nm_to_label = {}
    for d in cfg['all_maps']:
        map_nm_to_label.update(d)
        for k in d:
            map_names.append(k)
    print(Fore.GREEN + "Maps to compare:\n- {}".format('\n- '.join(map_names)))
    print(Fore.GREEN + "Labels:\n{}".format(map_nm_to_label))

    fim_map_nms = [v for v in map_names if 'info' in v]
    compare_orient_map_nms = [v for v in map_names if 'info' in v]
    compare_cont_motion_map_nms = [v for v in map_names if 'info' in v]
    print("Will analyze FIM for {}".format(fim_map_nms))
    print("Will compare orientations for {}".format(compare_orient_map_nms))
    print("Will compare cont. motion for {}".format(compare_cont_motion_map_nms))

    save_dir = os.path.join(args.res_dir, args.save_dir)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    else:
        print(Fore.RED + "Save folder exists, will re-use and overwrite.")
    print("Going to save to {}".format(save_dir))

    map_nm_to_res = {}
    for map_nm in map_names:
        print(Fore.GREEN + "====> Reading {}...".format(map_nm))
        map_nm_to_res[map_nm] = readResults(args.res_dir, map_nm)

    print(Fore.YELLOW + Style.BRIGHT + "Start analysis.")

    print(Fore.GREEN + "1. Table of complexity.")
    _writeComplexityTable(map_nm_to_res, args.pc_res_key, map_names, map_nm_to_label,
                          os.path.join(save_dir, 'complexity_table.txt'))

    print(Fore.GREEN + "2. FIM difference.")
    map_nm_to_fim_diff_perc = _computeAndWriteFIMDiff(
        map_nm_to_res, fim_map_nms, map_nm_to_label, save_dir)
    _boxplotFIMDiffs(map_nm_to_fim_diff_perc, fim_map_nms, map_nm_to_label, save_dir)

    print(Fore.GREEN + "3. Optimal views.")
    map_nm_to_orient_diff = _computeAndWriteOptimalOrientDiff(
        map_nm_to_res, compare_orient_map_nms, map_nm_to_label, save_dir)
    _boxplotOrientDiffs(map_nm_to_orient_diff, compare_orient_map_nms, map_nm_to_label, save_dir)

    print(Fore.GREEN + "4. Continous motion.")
    _compareContinuousMotion(map_nm_to_res, compare_cont_motion_map_nms, map_nm_to_label, save_dir)

    print(Fore.GREEN + Style.BRIGHT + "Start processing specified subsets...")
    sel_dir = os.path.join(save_dir, 'selected_results')
    if not os.path.exists(sel_dir):
        os.makedirs(sel_dir)
    if 'sel_complexity_table_entries' in cfg:
        print(Fore.GREEN + "- complexity table")
        _writeComplexityTable(map_nm_to_res, args.pc_res_key, cfg['sel_complexity_table_entries'], map_nm_to_label, os.path.join(
            sel_dir, 'complexity_table.txt'))

    if 'sel_fro_norm_table_entries' in cfg:
        print(Fore.GREEN + "- FIM diff. table")
        sel_fim_nms = cfg['sel_fro_norm_table_entries']
        sel_nm_to_fim_diff = _computeAndWriteFIMDiff(
            map_nm_to_res, sel_fim_nms, map_nm_to_label, sel_dir)
        _boxplotFIMDiffs(sel_nm_to_fim_diff, sel_fim_nms, map_nm_to_label, sel_dir)

    if 'sel_hist_orient_entries' in cfg:
        sel_orient_nms = cfg['sel_hist_orient_entries']
        print(Fore.GREEN + "- Orientation diff.")
        sel_nm_to_orient_diff = _computeAndWriteOptimalOrientDiff(
            map_nm_to_res, sel_orient_nms, map_nm_to_label, sel_dir)
        _boxplotOrientDiffs(sel_nm_to_orient_diff, sel_orient_nms, map_nm_to_label, sel_dir)

    if 'sel_cont_motion_plot' in cfg:
        print(Fore.GREEN + "- continuous motion")
        _compareContinuousMotion(
            map_nm_to_res, cfg['sel_cont_motion_plot'], map_nm_to_label, sel_dir)
