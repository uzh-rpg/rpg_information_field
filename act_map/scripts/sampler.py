#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np

# import healpy as hp

import visibility as vis


def sampleSphereUniformAngle(n_polar_samples, n_azimuth_samples):
    '''
    This function samples bearing vectors from a unit sphere
    by uniformly sampling the polar and azimuth angles.
    The vector at the north and south poles will always be returned.
    In addition, we take samples of polar (0 ~ pi) and azimuth (0 ~ 2pi).
    
    The input should be positive integers.
    The output is a Nx3 numpy array for all bearing vectors, 
    where the first and last ones are north and south poles

    '''

    assert n_polar_samples >= 1
    assert n_azimuth_samples >= 2

    bearing_vectors = [[0, 0, 1]]

    polar_interval = np.pi / (1 + n_polar_samples)
    azimuth_interval = 2 * np.pi / n_azimuth_samples

    polar_angles = [(v+1) * polar_interval for v in range(n_polar_samples)]
    azimuth_angles = [v * azimuth_interval for v in range(n_azimuth_samples)]

    for p_angle in polar_angles:
        for a_angle in azimuth_angles:
            x = np.sin(p_angle) * np.cos(a_angle)
            y = np.sin(p_angle) * np.sin(a_angle)
            z = np.cos(p_angle)
            bearing_vectors.append([x, y, z])

    bearing_vectors.append([0, 0, -1])

    return np.array(bearing_vectors)


# def sampleSphereHEALPix(nside):
    # '''
    # https://healpix.jpl.nasa.gov/
    # The number of points will be 12 * nsides^2
    # '''
    # vecs = hp.pix2vec(nside, np.arange(hp.nside2npix(nside)))
    # vecs = np.array(vecs)
    # return vecs.transpose()


def sampleSphereUniformArea(num_pts):
    '''
    http://mathworld.wolfram.com/SpherePointPicking.html
    Code from
    https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
    '''

    bearing_vectors = np.zeros((num_pts, 3))

    indices = np.arange(0, num_pts, dtype=float) + 0.5

    phi = np.arccos(1 - 2*indices/num_pts)
    theta = np.pi * (1 + 5**0.5) * indices

    x, y, z =\
        np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)

    bearing_vectors[:, 0] = x
    bearing_vectors[:, 1] = y
    bearing_vectors[:, 2] = z

    return bearing_vectors


def randomPointsOnSphere(num_pts, radius):
    '''
    generate random points on a sphere of size radius
    return N x 3 array
    '''

    assert num_pts >= 1 and radius > 0

    rand_pts = np.random.uniform(-1, 1, [num_pts, 3])
    rand_pts = np.array([radius * v / np.linalg.norm(v) for v in rand_pts])

    return rand_pts


def nearestDistances(fs):
    assert fs.shape[1] == 3
    n_fs = fs.shape[0]
    min_dist = np.zeros(n_fs)

    for i in range(n_fs):
        min_d = 1e5
        cur_f = fs[i]
        for j in range(n_fs):
            if i == j:
                continue
            d = np.linalg.norm(cur_f - fs[j])
            min_d = min(min_d, d)
        min_dist[i] = min_d

    return min_dist


def nearestAngleRad(fs):
    assert fs.shape[1] == 3
    n_fs = fs.shape[0]
    min_angle = np.zeros(n_fs)

    for i in range(n_fs):
        min_rad = 1e5
        cur_f = fs[i]
        for j in range(n_fs):
            if i == j:
                continue
            rad = vis.angleRadBetween(cur_f, fs[j])
            min_rad = min(min_rad, rad)
        min_angle[i] = min_rad

    return min_angle


if __name__ == '__main__':
    pass
