#!/usr/bin/env python2

import os
import argparse
import shutil
import subprocess
import shlex
from colorama import init, Fore, Style
import time

init(autoreset=True)

kLaunchToFolder = {"gp_info_map_warehouse.launch": "gp_info",
                   "gp_trace_map_warehouse.launch": "gp_trace",
                   "quadratic_info_map_warehouse.launch": 'quad_info',
                   "quadratic_trace_map_warehouse.launch": "quad_trace"
                   }
kLaunchToFnPref = {"gp_info_map_warehouse.launch": "gp_info",
                   "gp_trace_map_warehouse.launch": "gp_trace",
                   "quadratic_info_map_warehouse.launch": 'quad_info',
                   "quadratic_trace_map_warehouse.launch": "quad_trace"
                   }
kMapSuf = ["r1_a30", "r2_a20"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--out_topdir', required=True)
    parser.add_argument('--overwrite', dest='overwrite', action='store_true')
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    launch_fns = sorted(list(kLaunchToFolder.keys()))
    for lfn in launch_fns:
        assert lfn in kLaunchToFnPref

    print(Fore.YELLOW + "Launch files: {}".format(launch_fns))
    print(Fore.YELLOW + "Maps: {}".format(kMapSuf))
    print(Fore.YELLOW + "==> {}".format(args.out_topdir))

    for ln in launch_fns:
        for mp in kMapSuf:
            out_dir = os.path.join(args.out_topdir, kLaunchToFolder[ln] + "_" + mp)
            launch_cmd = "roslaunch act_map_exp {} map_suf:={}".format(ln, mp)
            print(Fore.GREEN + "Cmd: {}\nOut: {}".format(launch_cmd, out_dir))
            if os.path.exists(out_dir):
                if args.overwrite:
                    print(Fore.RED + "removing existing folder...")
                    shutil.rmtree(out_dir)
                else:
                    print(Fore.RED + "Result exists and no overwrite, continue...")
                    continue
            os.makedirs(out_dir)

            print(Style.BRIGHT + Fore.GREEN + "Launching...")
            p = subprocess.Popen(shlex.split(launch_cmd))
            time.sleep(15)

            print(Style.BRIGHT + Fore.GREEN + "Recomputing ...")
            recompute_cmd = "rosservice call /act_map/recompute_kernel_layer"
            print(Fore.BLUE + recompute_cmd)
            subprocess.call(shlex.split(recompute_cmd))

            print(Style.BRIGHT + Fore.GREEN + "Saving ...")
            save_cmd = """rosservice call /act_map/save_act_map_layers "file_path: '{}'" """.format(os.path.abspath(out_dir))
            print(Fore.BLUE + save_cmd)
            subprocess.call(shlex.split(save_cmd))

            print(Style.BRIGHT + Fore.GREEN + "Killing the node ...")
            p.kill()
