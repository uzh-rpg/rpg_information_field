#!/usr/bin/env python2

import numpy as np
import os
import StringIO

from colorama import Fore, init

init(autoreset=True)


# https://gist.github.com/edz-o/84d63fec2fc2d70721e775337c07e9c9
def DepthConversion(PointDepth, f):
    H = PointDepth.shape[0]
    W = PointDepth.shape[1]
    i_c = np.float(H) / 2 - 1
    j_c = np.float(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(
        0, W-1, num=W), np.linspace(0, H-1, num=H))
    DistanceFromCenter = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    PlaneDepth = PointDepth / (1 + (DistanceFromCenter / f)**2)**(0.5)
    return PlaneDepth


def readCameraIntri(ini_fn):
    assert os.path.exists(ini_fn)
    import configparser
    cfg = configparser.ConfigParser()
    cfg.read(ini_fn)
    unreal_cv_k = 'UnrealCV.Core'
    assert unreal_cv_k in cfg.sections()
    cam_intri = cfg[unreal_cv_k]
    return {'width': int(cam_intri['Width']),
            'height': int(cam_intri['Height']),
            'horizontal_fov': float(cam_intri['FOV'])}


def setCameraIntri(client, w, h, wfov_deg, cam_id=0):
    assert False, """use command line to set intrisics seesm not reliable, 
    use the configuration file instead"""
    assert client.isconnected()

    res_cmd = "vrun r.setres {}x{}".format(int(w), int(h))
    print(Fore.BLUE + ">> " + res_cmd)
    res_size = client.request(res_cmd)
    print('  {}'.format(res_size))

    fov_cmd = "vset /camera/{}/horizontal_fieldofview {}".format(
        cam_id, wfov_deg)
    print(Fore.BLUE + ">> " + fov_cmd)
    res_fov = client.request(fov_cmd)
    print('  {}'.format(res_fov))


def saveImage(client, img_nm, cam_id=0, quiet=True):
    cap_cmd = 'vget /camera/{}/lit {}'.format(cam_id, img_nm)
    if not quiet:
        print(Fore.BLUE + ">> " + cap_cmd)
    res = client.request(cap_cmd)
    if not quiet:
        print('  {}'.format(res))


def saveDepth(client, depth_fn, focal, cam_id=0, quiet=True, zdepth=True):
    assert depth_fn.endswith('.npy')
    cap_cmd = 'vget /camera/{}/depth npy'.format(cam_id)
    if not quiet:
        print(Fore.BLUE + ">> " + cap_cmd)
    depth = np.load(StringIO.StringIO(client.request(cap_cmd)))
    if zdepth:
        depth = DepthConversion(depth, focal)
    if not quiet:
        print("Depth map shape: {}".format(depth.shape))
    np.save(depth_fn, depth)

    return depth


def setCameraPose(client, xyz, pyr, cam_id=0, quiet=True):
    set_cmd = 'vset /camera/{}/pose {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}'.format(
        cam_id, xyz[0], xyz[1], xyz[2], pyr[0], pyr[1], pyr[2])
    if not quiet:
        print(Fore.BLUE + ">> " + set_cmd)
    res = client.request(set_cmd)
    if not quiet or res != "ok":
        print('  {}'.format(res))


def getUEPose(client, cam_id=0):
    pose_cmd = 'vget /camera/{}/pose'.format(cam_id)
    print(Fore.BLUE + ">> " + pose_cmd)
    xyzpyr = client.request(pose_cmd)
    print('  {}'.format(xyzpyr))

    return xyzpyr


def getUnrealcvStatus(client):
    stat = client.request('vget /unrealcv/status')
    return stat


def readUnrealPoses(fn):
    times = []
    xyzpyr = []
    with open(fn) as f:
        for l in f:
            if l.startswith('#'):
                continue
            items = l.strip().split()

            assert len(items) == 6 or len(items) == 7

            if len(items) == 7:
                times.append(items[0])
                pose_i = [float(items[i]) for i in range(1, 7)]
            else:
                pose_i = [float(items[i]) for i in range(0, 6)]

            xyzpyr.append(pose_i)
    return times, xyzpyr


def focalLength(width, wfov_deg):
    half_wfov_rad = 0.5 * np.deg2rad(wfov_deg)
    return (width / 2.0) / np.tan(half_wfov_rad)
