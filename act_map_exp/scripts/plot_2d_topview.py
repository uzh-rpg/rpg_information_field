#!/usr/bin/env python2

import os
import argparse

import numpy as np
from matplotlib import rc
import matplotlib.pyplot as plt
from colorama import init, Fore
from PIL import Image
import yaml

init(autoreset=True)

rc('font', **{'serif': ['Cardo'], 'size': 25})
rc('text', usetex=True)


def _xyToRGB(u, v):
    ang = int(np.rad2deg(np.arctan2(u, v)))
    if ang == 0:
        return [255, 0, 0]
    elif ang == 120:
        return [0, 255, 0]
    elif ang == -120:
        return [0, 0, 255]

    if ang > 0 and ang < 120:
        k = ang / 120.0
        return [int((1-k) * 255), int(k*255), 0]

    if ang < 0 and ang > -120:
        k = (0-ang) / 120.0
        return [int((1-k) * 255), 0, int(k*255)]

    if ang > 120 and ang <= 180:
        k = (ang - 120.0) / 120.0
        return [0, int((1-k) * 255), int(k * 255)]
    if ang < -120:
        k = (- 120.0 - ang) / 120.0
        return [0, int(k * 255), int((1-k) * 255)]


def _readTwcsSingle(Twcs_fn):
    Twcs_flat = np.loadtxt(Twcs_fn)
    assert Twcs_flat.shape[1] == 16 + 1
    return [np.array(v.ravel().tolist()[1:]).reshape((4, 4)) for v in Twcs_flat]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--points3d", required=True)
    parser.add_argument("--views", required=True)

    parser.add_argument('--plot_cfg', type=str, default=None)
    parser.add_argument("--Twcs", type=str, default=None)

    parser.add_argument('--top_res_dir', required=True)
    parser.add_argument('--out_dir', type=str, default=None)
    parser.add_argument('--max_z', default=5, type=float)
    parser.add_argument('--view_color_code', action='store_true', dest='view_color_code')
    parser.set_defaults(view_color_code=False)
    args = parser.parse_args()
    print(args.__dict__)

    assert args.plot_cfg is not None or args.Twcs is not None

    out_dir = args.out_dir if args.out_dir else args.top_res_dir

    raw_pts3d = np.loadtxt(args.points3d)
    assert raw_pts3d.shape[1] == 3
    n_raw_pts = raw_pts3d.shape[0]
    raw_views = np.loadtxt(args.views)
    assert raw_views.shape[0] == n_raw_pts
    print(Fore.YELLOW + "Read {} landmarks.".format(n_raw_pts))

    all_Twcs = []
    all_colors = []
    all_labels = []
    if args.plot_cfg:
        assert os.path.exists(args.plot_cfg)
        print(Fore.YELLOW + "Read multiple Twcs from {}".format(args.plot_cfg))
        with open(args.plot_cfg, 'r') as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)
        assert type(cfg) == list
        for cfg_i in cfg:
            assert len(cfg_i) == 1
            for res_i, settings_i in cfg_i.items():
                print("- read {} with settings {}".format(res_i, settings_i))
                all_Twcs.append(_readTwcsSingle(
                    os.path.join(args.top_res_dir, res_i, 'stamped_Twc.txt')))
                all_colors.append(settings_i['color'])
                label_i = settings_i['label']
                if label_i == 'None':
                    all_labels.append(None)
                else:
                    all_labels.append(label_i)
    else:
        print(Fore.YELLOW + "Read single Twcs from {}".format(args.Twcs))
        all_Twcs.append(_readTwcsSingle(args.Twcs))
        all_colors.append('seagreen')
        all_labels.append('rrt')

    # filter and preprocess (swap x y, and invert the swapped y)
    print("Preprocess landmarks...")
    pts3d = []
    views = []
    for idx in range(n_raw_pts):
        pt_i = raw_pts3d[idx]
        view_i = raw_views[idx]
        if pt_i[2] > args.max_z:
            continue
        pts3d.append([pt_i[1], -pt_i[0], pt_i[2]])
        views.append([view_i[1], -view_i[0], view_i[2]])
    pts3d = np.array(pts3d)
    views = np.array(views)

    print("Preprocess camera views...")
    all_cam_views_2d = []
    all_cam_pos_2d = []
    all_start_pos = []
    for Twcs in all_Twcs:
        cam_views_2d = []
        cam_pos_2d = []
        for Twc in Twcs:
            cam_view_i = np.dot(Twc[0:3, 0:3], [0.0, 0.0, 1.0])
            cam_view_2d_i = np.array([cam_view_i[1], -cam_view_i[0]])
            cam_view_2d_i = 0.2 * cam_view_2d_i / np.linalg.norm(cam_view_2d_i)
            cam_views_2d.append(cam_view_2d_i.ravel().tolist())
            cam_pos_i = Twc[0:3, 3]
            cam_pos_2d.append([cam_pos_i[1], -cam_pos_i[0]])
        all_start_pos.append(cam_pos_2d[0])
        all_cam_views_2d.append(np.array(cam_views_2d))
        all_cam_pos_2d.append(np.array(cam_pos_2d))
    aver_start = np.mean(np.array(all_start_pos), axis=0)

    # colormap 
    dim = 500
    center = 250
    rad = 250
    img = np.zeros((500, 500, 4), 'uint8')
    for ri in range(dim):
        for ci in range(dim):
            x = ci - center
            y = -(ri - center)
            if np.sqrt(x*x + y*y) >= rad:
                img[ri, ci, :] = [0, 0, 0, 0]
                continue
            img[ri, ci, :] = _xyToRGB(x, y) + [255]
    colormap = Image.fromarray(img)
    colormap = colormap.convert('RGBA')
    savecolormap = Image.new('RGBA', (dim, dim), (0, 0, 0, 0))
    savecolormap.paste(colormap, (0, 0))
    colormap_fn = os.path.join(out_dir, 'colormap.png')
    savecolormap.save(colormap_fn)
    colormap_plt = plt.imread(colormap_fn)


    # plot
    print("Plotting...")
    n_pts = pts3d.shape[0]
    pts_colors = []
    for idx in range(n_pts):
        if args.view_color_code:
            c = _xyToRGB(views[idx][0], views[idx][1])
        else:
            c = [0, 0, 255]
        c_str = "#{:02x}{:02x}{:02x}".format(c[0], c[1], c[2])
        pts_colors.append(c_str)
    #
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_subplot(111)
    ax.scatter(pts3d[:, 0], pts3d[:, 1], c=pts_colors)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.axis('off')

    # insert colormap for points
    if args.view_color_code:
        cm_ax = fig.add_axes([0.01, 0.81, 0.15, 0.15], anchor='SW')
        cm_ax.imshow(colormap_plt)
        cm_ax.axis('off')

    for cam_views_2d, cam_pos_2d, color, label in zip(
            all_cam_views_2d, all_cam_pos_2d, all_colors, all_labels):
        ax.quiver(cam_pos_2d[:, 0], cam_pos_2d[:, 1], cam_views_2d[:, 0], cam_views_2d[:, 1],
                  width=0.003, headwidth=3, headlength=5, headaxislength=4, scale=6,
                  scale_units='width', color=color, label=label)
        ax.plot(cam_pos_2d[:, 0], cam_pos_2d[:, 1],
                color=color, linestyle='--')
    # ax.text(aver_start[0], aver_start[1] - 1.5, 'start', color='darkgreen',
    #         backgroundcolor='lightgray')
    ax.legend(ncol=len(cam_views_2d), loc='upper center')


    plt.axis('equal')
    plt.tight_layout()
    fig.savefig(os.path.join(out_dir, '2d_top_view.png'),
                bbox_inches='tight')

