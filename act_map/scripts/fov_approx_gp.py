#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import GPy

import visibility as vis
import conversion as conv
import sampler as sa

_save_ext = ".pdf"

rc('font', **{'family': 'serif', 'serif': ['Cardo'], 'size': 25})
rc('text', usetex=True)


def trainGPFoVApproximatorSingle(unit_bearings, hfov_rad, pw,
                                 sigmoid_k=100, opt_restart=-1, ker_nm='rbf',
                                 fix_length_scale=None,
                                 fix_gaussian_noise=None,
                                 use_fast_sigmoid=True):
    '''
    Learn a GP to approximate the visibility of one specific point.
    '''
    assert unit_bearings.shape[1] == 3
    assert pw.shape == (3,)

    soft_vis_fs = np.array([vis.softVisibility(v, pw, hfov_rad, sigmoid_k,
                                               use_fast_sigmoid)
                            for v in unit_bearings])
    soft_vis_fs = soft_vis_fs.reshape(-1, 1)
    ker = None
    if ker_nm is 'rbf':
        ker = GPy.kern.RBF(input_dim=3)
        if fix_length_scale is not None:
            ker.lengthscale = fix_length_scale
            ker.lengthscale.constrain_fixed()
    elif ker_nm is 'poly':
        ker = GPy.kern.Poly(input_dim=3)
    elif ker_nm is 'mlp':
        ker = GPy.kern.MLP(input_dim=3)
    else:
        assert False, "Unknown kernel type."

    m = GPy.models.GPRegression(unit_bearings, soft_vis_fs, ker)
    if fix_gaussian_noise is not None and type(fix_gaussian_noise) is float:
        m.Gaussian_noise.variance = fix_gaussian_noise
        m.Gaussian_noise.variance.constrain_fixed()
    m.optimize()
    if opt_restart is not None and opt_restart > 1:
        m.optimize_restarts(num_restarts=opt_restart)

    return m


def trainGPFoVApproximatorSingleSphereCoord(
        azimuth_polar, hfov_rad, pw,
        sigmoid_k=100, opt_restart=-1, ker_nm='rbf',
        fix_length_scale=None,
        fix_gaussian_noise=None,
        use_fast_sigmoid=True):
    '''
    Using azimuth and polar angles as representation
    '''
    assert azimuth_polar.shape[1] == 2
    assert pw.shape == (3,)

    unit_bearings = conv.azimuthPolarToFs(azimuth_polar)
    assert unit_bearings.shape[1] == 3
    soft_vis_fs = np.array([vis.softVisibility(v, pw, hfov_rad, sigmoid_k,
                                               use_fast_sigmoid)
                            for v in unit_bearings])
    soft_vis_fs = soft_vis_fs.reshape(-1, 1)

    ker_rbf = GPy.kern.RBF(input_dim=2, ARD=True)
    if fix_length_scale is not None:
        ker_rbf['.*lengthscale'] = fix_length_scale
        ker_rbf['.*lengthscale'].constrain_fixed()

    ker = None
    if ker_nm is 'rbf':
        print("Using RBF kernel with ARD.")
        ker = ker_rbf
    elif ker_nm is 'rbf_period':
        print("Using RBF kernel with ARD * period")
        ker_per = GPy.kern.StdPeriodic(input_dim=1, active_dims=[0],
                                       period=2*np.pi)
        ker_per.period.constrain_fixed()

        ker = ker_rbf * ker_per
    else:
        assert False, "Unknown kernel type."

    m = GPy.models.GPRegression(azimuth_polar, soft_vis_fs, ker)
    if fix_gaussian_noise is not None and type(fix_gaussian_noise) is float:
        m.Gaussian_noise.variance = fix_gaussian_noise
        m.Gaussian_noise.variance.constrain_fixed()
    m.optimize()
    if opt_restart is not None and opt_restart > 1:
        m.optimize_restarts(num_restarts=opt_restart)

    return m


def trainGPFoVApproximator(unit_bearings, hfov_rad,
                           sigmoid_k=100, opt_restart=-1, ker='rbf'):
    '''
    Learn a GP to approximate the visibility of a field-of-view using randomly
    generated points.
    '''
    pass


class FoVApproximatorGP(object):
    base_savedir_nm = 'fov_approximator_gp'

    sampled_fs_fn = 'sampled_fs.txt'

    fov_params_fn = 'fov_params.txt'

    kernel_params_fn = 'kernel_params.txt'

    '''
    load and save approximation files using Gaussian process
    also perform some easy computation for regressing
    Use the sum of RBF kernel and white noise kernel
    '''

    @staticmethod
    def _rbf(v1, v2, var, length):
        return var * np.exp(-0.5 * np.linalg.norm(v1 - v2)**2 / (length**2))

    def __init__(self, profile_dir=None):
        '''
        load from directory containing all necessary files
        '''
        self.sampled_fs = None
        self.sigmoid_k = None
        self.hfov_deg = None
        self.hfov_rad = None

        self.rbf_var = None
        self.rbf_lengthscale = None
        self.white_var = None

        self.fast_sigmoid = None

        self.K = None
        self.invK = None

        self.initialzied = False

        if profile_dir is not None:
            self.load(profile_dir)

    def computeKAndInvK(self):
        n_fs = self.sampled_fs.shape[0]
        self.K = np.zeros((n_fs, n_fs))
        for ri in range(n_fs):
            vri = self.sampled_fs[ri]
            for ci in range(ri, n_fs):
                vci = self.sampled_fs[ci]
                self.K[ri][ci] = self._rbf(vri, vci,
                                           self.rbf_var, self.rbf_lengthscale)
                if ri == ci:
                    self.K[ri][ci] += self.white_var
                else:
                    self.K[ci][ri] = self.K[ri][ci]
        self.invK = np.linalg.inv(self.K)

    def calculateY(self, pw):
        soft_vis = np.array(
            [vis.softVisibility(v, pw, self.hfov_rad, self.sigmoid_k,
                                self.fast_sigmoid)
             for v in self.sampled_fs])
        soft_vis = soft_vis.reshape((-1, 1))

        return soft_vis

    def calculateKreg(self, f):
        n_fs = self.sampled_fs.shape[0]
        Kreg = np.zeros((1, n_fs))
        for i in range(n_fs):
            Kreg[0][i] = self._rbf(f, self.sampled_fs[i],
                                   self.rbf_var, self.rbf_lengthscale)
        return Kreg

    def setApproximatorData(self, data_dict):
        self.sampled_fs = data_dict['sampled_fs']
        self.sigmoid_k = data_dict['sigmoid_k']
        self.hfov_deg = data_dict['hfov_deg']
        self.rbf_var = data_dict['rbf_var']
        self.rbf_lengthscale = data_dict['rbf_lengthscale']
        self.white_var = data_dict['white_var']

        self.computeKAndInvK()
        self.hfov_rad = np.deg2rad(self.hfov_deg)
        self.initialzied = True

    def load(self, data_dir):
        self.sampled_fs = np.loadtxt(
            os.path.join(data_dir, self.sampled_fs_fn))
        assert self.sampled_fs.shape[1] == 3
        d = np.ravel(np.loadtxt(os.path.join(data_dir, self.fov_params_fn)))
        assert d.shape[0] == 2
        self.sigmoid_k, self.hfov_deg = d[0], d[1]
        d = np.ravel(np.loadtxt(os.path.join(data_dir, self.kernel_params_fn)))
        assert d.shape[0] == 3
        self.rbf_var, self.rbf_lengthscale, self.white_var = \
            d[0], d[1], d[2]

        if data_dir.endswith('fast'):
            self.fast_sigmoid = True
        else:
            self.fast_sigmoid = False

        self.computeKAndInvK()
        self.hfov_rad = np.deg2rad(self.hfov_deg)
        self.initialzied = True

    def save(self, save_dir):
        '''
        save essential files to folder
        '''
        assert os.path.exists(save_dir)
        np.savetxt(os.path.join(save_dir, self.sampled_fs_fn),
                   self.sampled_fs)
        np.savetxt(os.path.join(save_dir, self.fov_params_fn),
                   np.array([self.sigmoid_k, self.hfov_deg]),
                   header='sigmoid_k hfov_deg')
        np.savetxt(os.path.join(save_dir, self.kernel_params_fn),
                   np.array([self.rbf_var, self.rbf_lengthscale,
                             self.white_var]),
                   header='rbf_var rbf_lengthscale white_var')

    @staticmethod
    def uniqueName(hfov_deg, n_induced_fs, n_train_landmarks, sigmoid_k,
                   fast_sigmoid,
                   extra=[]):
        str_list = ['fov'+str(hfov_deg),
                    'fs'+str(n_induced_fs),
                    'lm'+str(n_train_landmarks),
                    'k'+str(sigmoid_k)]
        if fast_sigmoid:
            str_list.append('fast')
        for v in extra:
            assert type(v) == str

        str_list.extend(extra)

        return '_'.join(str_list)


if __name__ == '__main__':

    dir_path = os.path.dirname(os.path.realpath(__file__))
    plots_path = os.path.join(dir_path, 'plots')
    assert os.path.exists(plots_path), "{0} does not exist.".format(plots_path)
