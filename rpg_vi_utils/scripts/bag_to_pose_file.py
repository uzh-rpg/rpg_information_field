#!/usr/bin/env python

import os
import argparse

import numpy as np
import rosbag

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_fn', type=str, help='bag file name')
    parser.add_argument('topic', type=str, help='topic name (PoseStamped)')
    parser.add_argument('--wlast', action='store_true', default=False)
    parser.add_argument('--delim', type=str, default=",", help="delimiter")
    args = parser.parse_args()

    print("Arguments: {0}".format(args))

    bag_fn = os.path.basename(args.bag_fn)
    abs_bag_fn = os.path.abspath(args.bag_fn)
    abs_folder = os.path.dirname(abs_bag_fn)
    save_fn = os.path.join(abs_folder,
                           bag_fn.split('.')[0] +
                           args.topic.replace('/', '_') + '.txt')

    assert os.path.exists(abs_bag_fn)
    print("Going to extract {0} in {1} and save to {2}".format(args.topic,
                                                               abs_bag_fn,
                                                               save_fn))

    all_poses = []
    bag = rosbag.Bag(abs_bag_fn)
    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        time_nsec = msg.header.stamp.to_nsec()
        pose = msg.pose
        data = [time_nsec, pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.w, pose.orientation.x, pose.orientation.y,
                pose.orientation.z]
        all_poses.append(data)

    all_poses = np.array(all_poses)

    if args.wlast:
        all_poses_wlast = all_poses.copy()
        all_poses_wlast[:, 4:7], all_poses_wlast[:, 7] =\
            all_poses[:, 5:8], all_poses[:, 4]
        np.savetxt(save_fn, all_poses_wlast, delimiter=args.delim,
                   header="time_ns x y z qx qy qz qw")
    else:
        np.savetxt(save_fn, all_poses, delimiter=args.delim,
                   header="time_ns x y z qw qx qy qz")

