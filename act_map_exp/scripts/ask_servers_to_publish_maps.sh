#!/usr/bin/env bash
rosservice call /voxblox_node/publish_map
rosservice call /act_map/publish_act_map_layers
