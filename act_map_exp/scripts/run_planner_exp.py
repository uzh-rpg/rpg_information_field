#!/usr/bin/env python2

import os
import argparse
import shutil
import shlex
import subprocess
from colorama import init, Fore

import yaml

init(autoreset=True)


def _parseConfig(cfg_fn):
    assert os.path.exists(cfg_fn)
    assert cfg_fn.endswith('.yaml')
    cfg_dir = os.path.dirname(os.path.abspath(cfg_fn))

    with open(cfg_fn, 'r') as f:
        all_cfgs = yaml.load(f, Loader=yaml.FullLoader)
    print(all_cfgs)

    all_base_names = []
    all_base_fns = []
    all_var_fns = []

    for gk, cfg in all_cfgs.iteritems():
        print(Fore.YELLOW + "=====> Group {}".format(gk))
        bases = []
        variations = []
        base_names = []

        for b_i, n_i in cfg['base'].iteritems():
            bases.append(os.path.join(cfg_dir, b_i))
            base_names.append(n_i)
        for v in sorted(cfg['var']):
            variations.append(os.path.join(cfg_dir, v))

        print(Fore.YELLOW + "From configuraiton file {}:".format(cfg_fn))
        print(Fore.YELLOW + "- found {} bases".format(len(bases)))
        for base_idx, v in enumerate(bases):
            print("  - {}: {}".format(v, base_names[base_idx]))

        print(Fore.YELLOW + "- found {} variations".format(len(variations)))
        for v in variations:
            print("  - {}".format(v))
        all_base_fns.append(bases)
        all_base_names.append(base_names)
        all_var_fns.append(variations)

    return all_base_fns, all_base_names, all_var_fns


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config_yaml', type=str, help='yaml')

    parser.add_argument('--top_outdir', type=str, required=True,
                        help='where to put the output dir for the experiment.')

    parser.add_argument('--colmap_script_dir', required=True,
                        help='where to find the scripts to use COLMAP')

    parser.add_argument('--base_model', required=True,
                        help='colmap workspace against which to localize')

    parser.add_argument('--no_clear_output', action='store_false', dest='clear_output')

    parser.add_argument('--skip_plan', action='store_true', dest='skip_plan')
    parser.add_argument('--skip_render', action='store_true', dest='skip_render')
    parser.add_argument('--skip_reg', action='store_true', dest='skip_reg')
    parser.add_argument('--skip_eval', action='store_true', dest='skip_eval')

    parser.add_argument('--min_num_inliers', type=int, default=10)
    parser.add_argument('--max_trans_e_m', type=float, default=0.5)
    parser.add_argument('--max_rot_e_deg', type=float, default=10.0)

    parser.add_argument('--exp_nm_rm_sufix', type=str, default='_base')

    parser.set_defaults(clear_output=True, skip_plan=False, skip_render=False, skip_reg=False,
                        skip_eval=False)
    args = parser.parse_args()

    if args.skip_render or args.skip_plan or args.skip_reg:
        print(Fore.YELLOW + "Not clearing stuff due to skipping steps.")
        args.clear_output = False

    assert os.path.exists(args.colmap_script_dir)
    gen_list_sc = os.path.join(args.colmap_script_dir, 'generate_img_rel_path.py')
    assert os.path.exists(gen_list_sc)
    reg_sc = os.path.join(args.colmap_script_dir, 'register_images_to_model.py')
    assert os.path.exists(reg_sc)
    cal_e_sc = os.path.join(args.colmap_script_dir, 'calculate_pose_errors.py')
    assert os.path.exists(cal_e_sc)
    assert os.path.exists(args.base_model)
    base_img_dir = os.path.join(args.base_model, 'images')
    assert os.path.exists(base_img_dir)

    all_base_fns, all_base_names, all_var_fns = _parseConfig(args.config_yaml)

    for base_fns, base_names, var_fns in zip(all_base_fns, all_base_names, all_var_fns):
        print(Fore.YELLOW + "Processing group: base {} with var. {}".format(base_fns, var_fns))
        failed_cfgs = []
        for base_idx, base_f_i in enumerate(base_fns):
            print(Fore.RED + "==============================")
            print(Fore.RED + "===== Running base {}... =====".format(base_f_i))
            print(Fore.RED + "==============================")

            assert os.path.exists(base_f_i)
            exp_nm = base_names[base_idx]
            base_outdir_i = os.path.join(args.top_outdir, exp_nm)
            if not os.path.exists(base_outdir_i):
                os.makedirs(base_outdir_i)

            print(Fore.YELLOW + "Experiment {} from {} with variations: ".format(exp_nm, base_f_i))
            for v in var_fns:
                print("- {}".format(v))
            print("Results will be saved in {}".format(base_outdir_i))

            with open(base_f_i, 'r') as f:
                base_params = yaml.load(f, Loader=yaml.FullLoader)
            ana_cfg_fn = os.path.join(base_outdir_i, 'analysis_cfg.yaml')
            if os.path.exists(ana_cfg_fn):
                print(Fore.YELLOW + "Found prev. analysis_cfg.yaml, wil update it")
                with open(ana_cfg_fn, 'r') as f:
                    prev_ana_cfg = yaml.load(f, Loader=yaml.FullLoader)
                    var_dir_to_types = prev_ana_cfg['types']
                    print("- loaded types {}".format(var_dir_to_types))
            else:
                var_dir_to_types = {}
            for var_f_i in var_fns:
                print(Fore.RED + "===== Running variation {}... =====".format(var_f_i))

                with open(var_f_i) as fvar:
                    var_opts_i = yaml.load(fvar, Loader=yaml.FullLoader)
                var_nm_i = exp_nm + '_' + var_opts_i['type']
                outdir_i = os.path.abspath(os.path.join(base_outdir_i, var_nm_i))
                if args.clear_output and os.path.exists(outdir_i):
                    shutil.rmtree(outdir_i)
                    print(Fore.RED + "Removed {}".format(outdir_i))
                if not os.path.exists(outdir_i):
                    os.makedirs(outdir_i)
                print("- output: {}".format(outdir_i))
                var_dir_to_types[var_nm_i] = var_opts_i['type']

                print(Fore.YELLOW + "Step 1: plan and save")
                if not args.skip_plan:
                    cfg_i = os.path.join(outdir_i, var_nm_i+".yaml")
                    params_i = base_params.copy()
                    params_i.update(var_opts_i)
                    params_i['save_traj_abs_dir'] = outdir_i
                    params_i['save_abs_dir'] = outdir_i
                    with open(cfg_i, 'w') as f:
                        yaml.dump(params_i, f, default_flow_style=False)
                    print("- cfg: {}".format(cfg_i))
                    set_planner_cmd = ("""rosservice call /{}/set_planner_state"""
                                       """ "config: '{}'" """.format(var_opts_i['node'], cfg_i))
                    print(Fore.BLUE + set_planner_cmd)
                    subprocess.call(shlex.split(set_planner_cmd))
                    call_planner_cmd = "rosservice call /{}/plan_vis_save".format(var_opts_i['node'])
                    print(Fore.BLUE + call_planner_cmd)
                    subprocess.call(shlex.split(call_planner_cmd))

                    reset_planner_cmd = "rosservice call /{}/reset_planner".format(var_opts_i['node'])
                    print(Fore.BLUE + reset_planner_cmd)
                    subprocess.call(shlex.split(reset_planner_cmd))
                else:
                    print(Fore.BLUE + "> Skip planner")

                print(Fore.YELLOW + "Step 2: render from the sampled poses")
                render_dir_i = os.path.join(outdir_i, 'rendering')
                if not args.skip_render:
                    ue_pose_fn = os.path.join(outdir_i, 'stamped_Twc_ue.txt')
                    if not os.path.exists(ue_pose_fn):
                        failed_cfgs.append(base_f_i + '-' + var_f_i)
                        print(Fore.RED + "Cannot find UE poses, plan failed? CONTINUE TO NEXT.")
                        continue
                    render_cmd = ("rosrun unrealcv_bridge render_from_poses.py {} --save_dir {}"
                                  " --save_sleep_sec 0.1").format(ue_pose_fn, render_dir_i)
                    print(Fore.BLUE + render_cmd)
                    subprocess.call(shlex.split(render_cmd))
                else:
                    print(Fore.BLUE + "> Skip rendering.")

                print(Fore.YELLOW + "Step 3: register images to colmap models")
                reg_img_dir_i = os.path.join(render_dir_i, 'images')
                if not args.skip_reg:
                    gen_list_cmd = "{} --base_dir {} --img_dir {} --img_nm_to_cam_list {}".format(
                        gen_list_sc, base_img_dir, reg_img_dir_i,
                        os.path.join(render_dir_i, 'img_nm_to_colmap_cam.txt'))
                    print(Fore.BLUE + gen_list_cmd)
                    subprocess.call(shlex.split(gen_list_cmd))
                    rel_image_list = os.path.join(render_dir_i, 'images/rel_img_path.txt')
                    rel_cam_list = os.path.join(render_dir_i, 'images/rel_img_nm_to_cam_list.txt')
                    assert os.path.exists(rel_image_list)
                    assert os.path.exists(rel_cam_list)
                    reg_cmd = ("{} {} --reg_name {} --reg_list_fn {} "
                               "--img_nm_to_colmap_cam_list {} --upref_no_time "
                               "--min_num_inliers {}").format(
                                   reg_sc, args.base_model, var_nm_i, rel_image_list, rel_cam_list,
                                   args.min_num_inliers)
                    print(Fore.BLUE + reg_cmd)
                    subprocess.call(shlex.split(reg_cmd))
                else:
                    print(Fore.BLUE + "> Skip image registration.")

                print(Fore.YELLOW + "Step 4: evaluate pose error")
                if not args.skip_eval:
                    reg_model_dir_i = os.path.join(args.base_model, var_nm_i+"_sparse")
                    assert os.path.exists(reg_model_dir_i)
                    eval_cmd = ("{} --reg_model_dir {} --reg_img_name_to_colmap_Tcw {}"
                                " --reg_img_dir {} --output_path {}").format(
                                    cal_e_sc, reg_model_dir_i,
                                    os.path.join(render_dir_i, 'img_name_to_colmap_Tcw.txt'),
                                    reg_img_dir_i, outdir_i)
                    print(Fore.BLUE + eval_cmd)
                    subprocess.call(shlex.split(eval_cmd))
                else:
                    print(Fore.BLUE + "> Skip evaluation.")
            with open(os.path.join(base_outdir_i, 'analysis_cfg.yaml'), 'w') as f:
                yaml.dump({'types': var_dir_to_types,
                           'max_trans_e_m': args.max_trans_e_m,
                           'max_rot_e_deg': args.max_rot_e_deg}, f, default_flow_style=False)

    print(Fore.RED + "Failed configurations:")
    for v in failed_cfgs:
        print(Fore.RED + v)
