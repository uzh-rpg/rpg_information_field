#!/usr/bin/env python2

import argparse
import os
import sys
import time
from colorama import init, Fore
import numpy as np
from datetime import datetime

import unrealcv

import add_path
import ue_conversions as uc
import unrealcv_utils as uu

init(autoreset=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out_dir', type=str, required=True)

    parser.add_argument('--Hz', type=float, default=10)
    parser.add_argument('--timeout', type=int, default=30)
    parser.add_argument('--save_pref', type=str, default='poses')
    args = parser.parse_args()

    assert os.path.exists(args.out_dir)
    ue_pose_fn = os.path.join(args.out_dir, '{}_xyzpyr_ue.txt'.format(args.save_pref))
    Twc_fn = os.path.join(args.out_dir, '{}_Twc.txt'.format(args.save_pref))
    uepose_f = open(ue_pose_fn, 'w')
    Twc_f = open(Twc_fn, 'w')
    save_int_ms = int(1.0 / args.Hz * 1000)

    client = unrealcv.Client(('127.0.0.1', 9000))
    client.connect()
    assert client.isconnected()
    st = uu.getUnrealcvStatus(client)
    print(Fore.GREEN + st)

    print(Fore.YELLOW + "Going to read poses from client every {} ms and save to {}.".format(
        save_int_ms, ue_pose_fn))

    save_cnt = 0
    positions = []
    try:
        for i in xrange(args.timeout * 1000, 0, -1):
            time.sleep(1e-3)
            if i % save_int_ms == 0:
                print('Save at {} s'.format(i / save_int_ms))

                xyzpyr_str = uu.getUEPose(client)
                xyzpyr = map(float, xyzpyr_str.split(' '))
                assert len(xyzpyr) == 6
                xyzpyr[0] /= 100.0
                xyzpyr[1] /= 100.0
                xyzpyr[2] /= 100.0

                Twc_ue = uc.xyzpyrToTwcUE(xyzpyr)
                Twc = uc.ueTwcToStandard(Twc_ue)
                print("Position (robotics convention) is at {}".format(Twc[0:3, 3]))
                positions.append(Twc[0:3, 3])

                Twc_f.write("{}\n".format(' '.join([str(v) for v in Twc.ravel().tolist()])))
                uepose_f.write('{}\n'.format(' '.join([str(v) for v in xyzpyr])))

                save_cnt += 1
    except KeyboardInterrupt:
        print('Stop recording.')

    print(Fore.GREEN + "{} poses in total.".format(save_cnt))
    uepose_f.close()
    Twc_f.close()

    positions = np.array(positions)
    print("The maximum values of positions: {}".format(np.amax(positions, axis=0)))
    print("The minimum values of positions: {}".format(np.amin(positions, axis=0)))
