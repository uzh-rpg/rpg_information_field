#!/usr/bin/env bash
rosservice call /voxblox_node/generate_mesh
rosservice call /voxblox_node/publish_pointclouds
