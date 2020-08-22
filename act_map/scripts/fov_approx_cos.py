#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import rc

_save_ext = ".pdf"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 25})
rc('text', usetex=True)


def cosAppZeroOne(sample_thetas):
    '''
    Fixed approximation scheme
    Simplelyfrom 0 to 1 approximation, cannot adapt to different FoVs
    '''
    ws = np.array([0.5 + 0.5 * np.cos(v) for v in sample_thetas])
    return ws


def cosRatioApp(samples_thetas, hfov, alpha, beta):
    '''
    First order approximation
    '''
    k1 = (alpha*beta - alpha) / (1 + np.cos(hfov) - 2 * beta)
    k2 = k1 + alpha
    print("k1 and k2 are {0} and {1}".format(k1, k2))
    return np.array([k1 * np.cos(v) + k2 for v in samples_thetas])


def secondOrderAppRatio(samples_thetas, hfov, bound_s=0.1):
    '''
    Second order approximation
    bound_s: the ratio from the value at hfov with respect to the center
    '''
    coshfov = np.cos(hfov)
    coshfov2 = coshfov * coshfov
    k2 = (bound_s - 0.5 * coshfov - 0.5) / (coshfov2 - 1)
    k1 = 0.5
    k0 = 0.5 - k1
    print("k2, k1, and k0 are {0}, {1} and {2}".format(k2, k1, k0))
    return np.array([k2 * np.cos(v) * np.cos(v) + k1 * np.cos(v) + k0
                     for v in samples_thetas]), k2, k1, k0


def secondOrderAppValue(samples_thetas, hfov, gamma=0.8, s_alpha=0.1):
    '''
    Second order approximation
    gamma: value at hfov
    s_alpha: value at hfov with respect to the center
    '''
    alpha = np.cos(hfov)
    alpha2 = alpha * alpha
    denom = alpha - 2 * gamma + 1
    k2 = s_alpha * denom / (2 * gamma - 2 * gamma * alpha2)
    k1 = k2 * (-alpha2 + 1) / (denom)
    k0 = k2 * (2 * gamma - alpha - alpha2) / (denom)
    print("k2, k1, and k0 are {0}, {1} and {2} for hfov {3}".format(
        k2, k1, k0, hfov))
    return np.array([k2 * np.cos(v) * np.cos(v) + k1 * np.cos(v) + k0
                     for v in samples_thetas]), k2, k1, k0


if __name__ == '__main__':

    dir_path = os.path.dirname(os.path.realpath(__file__))
    plots_path = os.path.join(dir_path, 'plots')
    assert os.path.exists(plots_path), "{0} does not exist.".format(plots_path)
    
    theta = np.arange(-np.pi, np.pi, 0.05)
    deg = np.array([np.rad2deg(v) for v in theta])

    # hfovs = [np.pi / 4, np.pi / 3.5, np.pi / 3.3, np.pi / 3.0,
    # np.pi / 2.7, np.pi / 2.5]
    hfovs = [np.pi / 4, np.pi / 3.3, np.pi / 3.0, np.pi / 2.5]
    scd_approxs_val_diff_fov = []
    ks_fov = []
    for hfovi in hfovs:
        scd_approx, k2, k1, k0 = secondOrderAppValue(
            theta, hfovi, gamma=0.8, s_alpha=0.8)
        scd_approxs_val_diff_fov.append(scd_approx)
        ks_fov.append([k2, k1, k0])

    scd_approxs_val_diff_max = []
    ks_max = []
    # srange = np.arange(0.2, 1.0, 0.1)
    srange = np.array([0.3, 0.5, 0.8])
    for s in srange:
        scd_approx, k2, k1, k0 = secondOrderAppValue(
            theta, hfovi, gamma=0.9, s_alpha=s)
        scd_approxs_val_diff_max.append(scd_approx)
        ks_max.append([k2, k1, k0])

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111)
    for v, n, k in zip(scd_approxs_val_diff_fov, hfovs, ks_fov):
        nm = r'$k_2$: {:.2f} $k_1$: {:.2f} $k_0$: {:.2f}'.format(
            k[0], k[1], k[2])
        plt.plot(deg, v, label=nm)

    for v, n, k in zip(scd_approxs_val_diff_max, hfovs, ks_max):
        nm = r'$k_2$: {:.2f} $k_1$: {:.2f} $k_0$: {:.2f}'.format(
            k[0], k[1], k[2])
        plt.plot(deg, v, label=nm)

    ax.set_xlim(-185, 185)
    ax.set_xticks(np.arange(-180, 181, 40))
    ax.set_ylim(0, 1.1)
    ax.set_xlabel(r'Angle $\theta$ between $\mathbf{f}$ and $\mathbf{e}_3$'
                  ' (deg)')
    ax.set_ylabel(r'Visiblity approximation $v(\theta)$')
    plt.tight_layout()
    plt.legend(bbox_to_anchor=(0.95, 0.9),
               bbox_transform=plt.gcf().transFigure)
    fig.savefig(os.path.join(plots_path, "fov_approx"+_save_ext),
                bbox_inches='tight')

    plt.show()
