#!/usr/bin/env python2

import argparse
import os
import sys
import time
from colorama import init, Fore
import numpy as np
from datetime import datetime
import shutil
from tqdm import tqdm
import matplotlib.pyplot as plt

from unrealcv import client

import add_path
import ue_conversions as uc
import unrealcv_utils as uu
import tf_utils as tu

init(autoreset=True)

img_counter = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('ue_pose_txt', type=str)

    parser.add_argument('--img_width', type=int, default=640)
    parser.add_argument('--img_height', type=int, default=480)
    parser.add_argument('--fov_deg', type=float, default=90.0)

    parser.add_argument('--save_sleep_sec', type=float, default=0.3)

    parser.add_argument('--top_save_dir', type=str, default=None,
                        help="top dir under which a stamped folder will be created")
    parser.add_argument('--save_dir', type=str, default=None,
                        help="directly specify save dir")

    parser.add_argument('--save_depth', action='store_true', dest='save_depth')
    parser.add_argument('--vis_depth', action='store_true', dest='vis_depth')
    parser.add_argument('--z_depth', action='store_true', dest='zdepth')
    parser.add_argument('--ray_depth', action='store_false', dest='zdepth')

    parser.set_defaults(save_depth=False, vis_depth=False, zdepth=True)
    args = parser.parse_args()
    print(args.__dict__)

    if args.vis_depth:
        assert args.save_depth

    if args.save_dir:
        save_dir = args.save_dir
    else:
        assert os.path.exists(args.top_save_dir)
        save_dir = os.path.join(args.top_save_dir, '{}_rec_img_pose'.format(
            datetime.now().strftime("%Y%m%d%H%M%S")))
    if os.path.exists(save_dir):
        shutil.rmtree(save_dir)
    os.makedirs(save_dir)
    print(Fore.YELLOW + "Going to render from {} and save in {}:".format(
        args.ue_pose_txt, save_dir))

    colmap_pose_fn = os.path.join(save_dir, 'img_name_to_colmap_Tcw.txt')
    ue_pose_fn = os.path.join(save_dir, 'ue_xyzpyr.txt')
    img_dir = os.path.join(save_dir, 'images')
    os.makedirs(img_dir)
    cam_fn = os.path.join(save_dir, 'img_nm_to_colmap_cam.txt')
    shutil.copy2(args.ue_pose_txt, ue_pose_fn)

    if args.save_depth:
        depth_dir = os.path.join(save_dir, 'depths')
        os.makedirs(depth_dir)
        if args.vis_depth:
            depth_vis_dir = os.path.join(depth_dir, 'vis')
            os.makedirs(depth_vis_dir)

    print(Fore.YELLOW + '- colmap poses: {}'.format(colmap_pose_fn))
    print(Fore.YELLOW + '- ue poses: {}'.format(ue_pose_fn))
    print(Fore.YELLOW + '- images: {}'.format(img_dir))
    print(Fore.YELLOW + '- intrinsics: {}'.format(cam_fn))

    client.connect()
    assert client.isconnected()
    st = uu.getUnrealcvStatus(client)
    print(Fore.GREEN + st)

    times, poses_ue = uu.readUnrealPoses(args.ue_pose_txt)
    print('Read {} Unreal poses.'.format(len(poses_ue)))

    print(Fore.RED + "Step 1: set camera intrinsics")
    uu.setCameraIntri(client, args.img_width, args.img_height, args.fov_deg)
    focal = uu.focalLength(args.img_width, args.fov_deg)
    print('- The focal length is {}px.'.format(focal))

    img_names = []
    poses_colmap = []
    intri_str_colmap = []
    print(Fore.RED + "Step 2: step and save images.")

    for idx, xyzpyr in enumerate(tqdm(poses_ue)):
        xyz = xyzpyr[0:3]
        pyr = xyzpyr[3:6]

        xyz_ue_scale = [v * 100 for v in xyz]
        uu.setCameraPose(client, xyz_ue_scale, pyr)
        time.sleep(args.save_sleep_sec)

        img_name_i = '{:05d}.png'.format(idx)
        img_names.append(img_name_i)
        img_name_i_abs = os.path.abspath(os.path.join(img_dir, img_name_i))
        uu.saveImage(client, img_name_i_abs)

        if args.save_depth:
            depth_fn_i_abs = os.path.abspath(os.path.join(depth_dir, '{:05d}.npy'.format(idx)))
            depth = uu.saveDepth(client, depth_fn_i_abs, focal, zdepth=args.zdepth)
            if args.vis_depth:
                vis_fn = os.path.join(depth_vis_dir, "{:05d}.png".format(idx))
                plt.imsave(vis_fn, depth)
                # fig = plt.figure()
                # ax = fig.add_subplot(111)
                # ax.imshow(depth, vmin=0.0, vmax=100.0)
                # plt.axis('off')
                # plt.tight_layout()
                # fig.savefig(vs_fn, bbox_inches='tight')
                # plt.close(fig)

        Twc_ue = np.eye(4)
        Twc_ue[0:3, 0:3] = uc.eulerToRotmatUE(pyr[2], pyr[0], pyr[1])
        Twc_ue[0:3, 3] = np.array(xyz)
        Twc = uc.ueTwcToStandard(Twc_ue)
        qtvec_i = tu.TwcToColmapQT(Twc)
        poses_colmap.append(qtvec_i)

        intri_str_i = 'PINHOLE {} {} {} {} {} {}'.format(
            args.img_width, args.img_height, focal, focal,
            args.img_width/2.0, args.img_height/2.0)
        intri_str_colmap.append(intri_str_i)

    with open(colmap_pose_fn, 'w') as f:
        for nm_i, qtvec_i in zip(img_names, poses_colmap):
            f.write('{} {}\n'.format(nm_i, ' '.join([str(v) for v in qtvec_i])))

    with open(cam_fn, 'w') as f:
        for nm_i, intri_i in zip(img_names, intri_str_colmap):
            f.write('{} {}\n'.format(nm_i, intri_str_i))
