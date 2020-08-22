#!/usr/bin/env python2

import numpy as np

import tf_utils as tu


def getT_wue_w():
    T_wue_w = np.eye(4)
    T_wue_w[1, 1] = -1
    return T_wue_w


def getT_w_wue():
    return np.linalg.inv(getT_wue_w())


def getT_c_cue():
    T_cue_c = np.zeros((4, 4))
    T_cue_c[0, 1] = 1
    T_cue_c[1, 2] = -1
    T_cue_c[2, 0] = 1
    T_cue_c[3, 3] = 1

    return T_cue_c


def getT_cue_c():
    return np.linalg.inv(getT_c_cue())


def DEP_eulerToRotmatUE(roll_deg, pitch_deg, yaw_deg):
    # the code below is from 
    # https://github.com/EpicGames/UnrealEngine/blob/release/Engine/Source/Runtime/Core/Public/Math/RotationTranslationMatrix.h#L32
    # but the inverse of the yaw angle is needed to make things work...
    assert False, "This implementation is wrong."
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(-yaw_deg)

    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)

    R = np.zeros((3, 3))

    R[0, 0] = cp * cy
    R[0, 1] = cp * sy
    R[0, 2] = sp

    R[1, 0] = sr * sp * cy - cr * sy
    R[1, 1] = sr * sp * sy + cr * cy
    R[1, 2] = - sr * cp

    R[2, 0] = - (cr * sp * cy + sr * sy)
    R[2, 1] = cy * sr - cr * sp * sy
    R[2, 2] = cr * cp

    return R


# consistent with https://github.com/EpicGames/UnrealEngine/blob/f794321ffcad597c6232bc706304c0c9b4e154b2/Engine/Source/Runtime/Core/Private/Math/UnrealMath.cpp#L540
def eulerToRotmatUE(roll_deg, pitch_deg, yaw_deg):
    R_roll = tu.RotMatX(-roll_deg)
    R_pitch = tu.RotMatY(-pitch_deg)
    R_yaw = tu.RotMatZ(yaw_deg)

    return np.linalg.multi_dot([R_yaw, R_pitch, R_roll])


def xyzpyrToTwcUE(xyzpyr):
    xyz = xyzpyr[0:3]
    pyr = xyzpyr[3:6]
    Twc_ue = np.eye(4)
    Twc_ue[0:3, 0:3] = eulerToRotmatUE(pyr[2], pyr[0], pyr[1])
    Twc_ue[0:3, 3] = np.array(xyz)

    return Twc_ue


def ueTwcToStandard(Twc_ue):
    return np.linalg.multi_dot([getT_w_wue(), Twc_ue, getT_cue_c()])


def standardTwcToUE(Twc):
    return np.linalg.multi_dot([getT_wue_w(), Twc, getT_c_cue()])


if __name__ == '__main__':
    pass
