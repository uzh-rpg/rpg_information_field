#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import numpy as np

hist_bins = np.arange(0, 180, 10).tolist()


def calAngleBetween(v1, v2):
    if (not np.all(np.isfinite(v1))) or (not np.all(np.isfinite(v2))):
        print('Skipped invalid.')
        return None
    e_rad = np.arccos(v1.dot(v2) / (np.linalg.norm(v1)*np.linalg.norm(v2)))
    return np.rad2deg(e_rad)


def calErrorViews(views1, views2):
    err = []
    for v1, v2 in zip(views1, views2):
        err.append(calAngleBetween(v1, v2))
    err_fix = []
    for v in err:
        if np.isfinite(v):
            err_fix.append(v)
        else:
            err_fix.append(0.0)
    return err_fix
