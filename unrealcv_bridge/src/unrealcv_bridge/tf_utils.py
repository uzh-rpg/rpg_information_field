#!/usr/bin/env python2

import numpy as np

# right hand convention


def RotMatX(deg):
    rad = np.deg2rad(deg)
    s, c = np.sin(rad), np.cos(rad)

    Rx = np.eye(3)
    Rx[1, 1] = c
    Rx[2, 2] = c
    Rx[1, 2] = -s
    Rx[2, 1] = s

    return Rx


def RotMatY(deg):
    rad = np.deg2rad(deg)
    s, c = np.sin(rad), np.cos(rad)

    Ry = np.eye(3)
    Ry[0, 0] = c
    Ry[2, 2] = c
    Ry[2, 0] = -s
    Ry[0, 2] = s

    return Ry


def RotMatZ(deg):
    rad = np.deg2rad(deg)
    s, c = np.sin(rad), np.cos(rad)

    Rz = np.eye(3)
    Rz[0, 0] = c
    Rz[1, 1] = c
    Rz[0, 1] = -s
    Rz[1, 0] = s

    return Rz


def qvec2rotmat(qvec):
    # qvec = qvec / np.linalg.norm(qvec)
    return np.array([
        [1 - 2 * qvec[2]**2 - 2 * qvec[3]**2,
         2 * qvec[1] * qvec[2] - 2 * qvec[0] * qvec[3],
         2 * qvec[3] * qvec[1] + 2 * qvec[0] * qvec[2]],
        [2 * qvec[1] * qvec[2] + 2 * qvec[0] * qvec[3],
         1 - 2 * qvec[1]**2 - 2 * qvec[3]**2,
         2 * qvec[2] * qvec[3] - 2 * qvec[0] * qvec[1]],
        [2 * qvec[3] * qvec[1] - 2 * qvec[0] * qvec[2],
         2 * qvec[2] * qvec[3] + 2 * qvec[0] * qvec[1],
         1 - 2 * qvec[1]**2 - 2 * qvec[2]**2]])


def rotmat2qvec(R):
    Rxx, Ryx, Rzx, Rxy, Ryy, Rzy, Rxz, Ryz, Rzz = R.flat
    K = np.array([
        [Rxx - Ryy - Rzz, 0, 0, 0],
        [Ryx + Rxy, Ryy - Rxx - Rzz, 0, 0],
        [Rzx + Rxz, Rzy + Ryz, Rzz - Rxx - Ryy, 0],
        [Ryz - Rzy, Rzx - Rxz, Rxy - Ryx, Rxx + Ryy + Rzz]]) / 3.0
    eigvals, eigvecs = np.linalg.eigh(K)
    qvec = eigvecs[[3, 0, 1, 2], np.argmax(eigvals)]
    if qvec[0] < 0:
        qvec *= -1
    return qvec


def TwcToColmapQT(Twc):
    qvec = rotmat2qvec(Twc[0:3, 0:3].transpose())
    tvec = -np.dot(Twc[0:3, 0:3].transpose(), Twc[0:3, 3])
    return qvec.tolist() + tvec.tolist()
