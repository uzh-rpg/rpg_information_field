- [Overview](#overview)
- [Implemented Classes related](#implemented-classes-related)
  - [`ActMapServer`](#actmapserver)
  - [`UEProvider`](#ueprovider)

# Overview
This package implements the ROS wrappers for the `act_map` package, including publishing/subsribing topics, parameter reading and visualization.

# Implemented Classes related

## `ActMapServer`

`ActMapServer` is a templated class that provides an interface for the information field. The most important functions are:

* build information field incrementally (as 3D features are added) or in a batch
* publish and receive the information field (as Voxblox layers) via ROS messages
* load and save the information field
* query the information at a given pose (e.g., used in planning)

Under the `src` folder, there are several specializations of the server class using different voxel types.

## `UEProvider`

`UEProvider` is an interface that talks with an UnrealCV server. It basically simulates a sensor that corresponds to the camera in Unreal Engine and publishes the following:

* The pose of the sensor as `tf` transformation
* The image of the sensor
* The depth (z-depth)

This can be used as the input to vision algorithms (e.g., mapping using a depth camera).