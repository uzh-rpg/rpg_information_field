#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np


def unitBearingToSphereCoordRad(f):
    assert f.shape == (3,)
    x, y, z = f[0], f[1], f[2]

    rxy = np.sqrt(x**2 + y**2)
    polar = np.arctan2(rxy, z)
    azimuth = np.arctan2(y, x)

    return [azimuth, polar]


def sphereCoordRadToUnitBearing(azimuth, polar):
    r = 1.0

    z = r * np.cos(polar)

    rxy = r * np.sin(polar)
    x = rxy * np.cos(azimuth)
    y = rxy * np.sin(azimuth)

    n = np.sqrt(x**2 + y**2 + z**2)

    return [x/n, y/n, z/n]


def fsToAzimuthPolar(fs):
    assert fs.shape[1] == 3
    return np.array([unitBearingToSphereCoordRad(v) for v in fs])


def azimuthPolarToFs(azimuth_polar):
    assert azimuth_polar.shape[1] == 2
    return np.array([sphereCoordRadToUnitBearing(v[0], v[1])
                     for v in azimuth_polar])


if __name__ == '__main__':
    f = np.random.rand(3)
    f = f / np.linalg.norm(f)

    azi_pol = unitBearingToSphereCoordRad(f)
    f_rec = sphereCoordRadToUnitBearing(azi_pol[0], azi_pol[1])

    print('Original and recovered:\n{0}\n{1}'.format(f, f_rec))

    np.testing.assert_almost_equal(f, f_rec)
