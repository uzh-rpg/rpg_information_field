#!/usr/bin/env python2

import os
import argparse

import numpy as np
from matplotlib import rc, cm, colors
import matplotlib.pyplot as plt
from colorama import init, Fore
from analyze_rrt import kSecPerOutIter

init(autoreset=True)

rc('font', **{'serif': ['Cardo'], 'size': 20})
rc('text', usetex=True)


def readRRTDataIter(verts_fn, edges_fn):
    verts = np.loadtxt(verts_fn)
    assert verts.shape[1] == 3
    verts_converted = np.zeros(verts.shape)
    verts_converted[:, 0] = verts[:, 1]
    verts_converted[:, 1] = -verts[:, 0]
    connected_vert_ids = np.loadtxt(edges_fn)
    assert connected_vert_ids.shape[1] == 2
    edges_start = np.array([verts_converted[int(idx[0]), 0:2]
                            for idx in connected_vert_ids])
    edges_end = np.array([verts_converted[int(idx[1]), 0:2]
                          for idx in connected_vert_ids])

    return verts_converted, edges_start, edges_end


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--res_dir', required=True)
    parser.add_argument('--out_dir', default=None, type=str)
    parser.add_argument('--plot_iter', type=int, default=10)
    parser.add_argument('--plot_iter_step', type=int, default=2)
    parser.add_argument("--points3d", required=True)
    parser.add_argument('--max_z', default=5, type=float)
    parser.add_argument('--no_legend', action='store_false', dest='legend')
    parser.add_argument('--colorbar', action='store_true', dest='colorbar')
    parser.set_defaults(legend=True, colorbar=False)
    args = parser.parse_args()

    out_dir = args.out_dir if args.out_dir else args.res_dir
    nm = os.path.basename(args.res_dir)

    rrt_edges_nms = sorted([v for v in os.listdir(
        args.res_dir) if v.startswith('rrt_edges_iter_')])
    rrt_edges_fns = [os.path.join(args.res_dir, v) for v in rrt_edges_nms]
    rrt_verts_nms = sorted([v for v in os.listdir(
        args.res_dir) if v.startswith('rrt_verts_iter_')])
    rrt_verts_fns = [os.path.join(args.res_dir, v) for v in rrt_verts_nms]

    n_iter = len(rrt_edges_fns)
    assert len(rrt_verts_fns) == n_iter

    print(Fore.YELLOW + "Found RRT data of {} iterations.".format(n_iter))

    print("Read RRT data...")
    verts = []
    edges_start = []
    edges_end = []
    for iter in range(args.plot_iter):
        verts_i, edge_start_i, edge_end_i = readRRTDataIter(
            rrt_verts_fns[iter], rrt_edges_fns[iter])
        print("- iter {}: {} vert.,  {} edges".format(
            iter, verts_i.shape[0], edge_start_i.shape[0]))
        verts.append(verts_i)
        edges_start.append(edge_start_i)
        edges_end.append(edge_end_i)

    print("Load and preprocess landmarks...")
    raw_pts3d = np.loadtxt(args.points3d)
    pts3d = []
    for idx in range(raw_pts3d.shape[0]):
        pt_i = raw_pts3d[idx]
        if pt_i[2] > args.max_z:
            continue
        pts3d.append([pt_i[1], -pt_i[0], pt_i[2]])
    pts3d = np.array(pts3d)

    print("Plot...")
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(111)
    ax.scatter(pts3d[:, 0], pts3d[:, 1], c='gray', s=2.0, alpha=0.5)
    ax.scatter([], [], c='gray', s=10.0, label='Landmarks')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.axis('off')
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    cmap = cm.get_cmap('jet')
    for iter in np.arange(0, args.plot_iter, args.plot_iter_step):
        print('- iter {}: {} verts'.format(iter, verts[iter].shape))
        c = cmap(iter * 1.0 / (args.plot_iter - 1))
        ax.scatter(verts[iter][:, 0], verts[iter]
                   [:, 1], c=c, s=0.1, zorder=-iter)

    label = 'RRT* Vertices'
    dummy_d = 5 * args.plot_iter
    dummy_offset = 1
    dummy_x = np.arange(xlim[0] - dummy_offset -
                        dummy_d, xlim[0] - dummy_offset + 1e-5, 5)
    dummy_y = np.arange(ylim[0] - dummy_offset -
                        dummy_d, ylim[0] - dummy_offset + 1e-5, 5)
    dummy_c = np.array([v*5 for v in range(args.plot_iter + 1)])
    sc_cb = ax.scatter(dummy_x, dummy_y, c=dummy_c, vmin=0, vmax= 5 *(args.plot_iter - 1),
                       s=10.0, cmap=cmap, label=label)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)

    if args.legend:
        ax.legend(loc='upper center', ncol=2, fontsize='medium')
    if args.colorbar:
        cb_ax = fig.add_axes([0.13, 0.15, 0.02, 0.70], anchor='C')
        fig.colorbar(sc_cb, cax=cb_ax)

    save_nm = os.path.join(out_dir, 'rrt_evolution_{}.png'.format(nm))
    print("Saving to {}...".format(save_nm))
    fig.savefig(save_nm, bbox_inches='tight')
