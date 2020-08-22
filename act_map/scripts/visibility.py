#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np


def cosBetween(v1, v2):
    nprod = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = np.dot(v1, v2) / nprod

    return cos


def angleRadBetween(v1, v2):
    angle = np.arccos(cosBetween(v1, v2))

    return angle


def angleDegBetween(v1, v2):
    return np.rad2deg(angleRadBetween(v1, v2))


def sigmoidVisibilityFromCosine(cos, hfov_cos, sigmoid_k):
    return 1 / (1 + np.exp(-sigmoid_k*(cos - hfov_cos)))


def fastSigmoidVisibilityFromCosine(cos, hfov_cos, sigmoid_k):
    x = sigmoid_k * (cos - hfov_cos)
    return 0.5 * (x / (1 + np.abs(x))) + 0.5


def softVisibility(cam_z_w, pt_w, hfov_rad, sigmoid_k, fast_sigmoid):
    '''
    use a sigmoid function for visiblity score
    '''
    cos = cosBetween(cam_z_w, pt_w)
    
    if fast_sigmoid:
        return fastSigmoidVisibilityFromCosine(cos,
                                               np.cos(hfov_rad), sigmoid_k)
    else:
        return sigmoidVisibilityFromCosine(cos, np.cos(hfov_rad), sigmoid_k)


def checkVisibility(cam_z_w, pt_w, hfov_rad):
    '''
    check the visibility of pt_w in a camera with optical axis as cam_z_w
    '''
    angle = angleRadBetween(cam_z_w, pt_w)

    return int(angle < 0.99 * hfov_rad)


if __name__ == '__main__':
    pass
